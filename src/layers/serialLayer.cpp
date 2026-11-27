#include "serialLayer.hpp"

#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>

volatile uint32_t SerialLayer::s_rxOverflows = 0;

static uint8_t g_bridgeRxRing[4096];

// Drain thread of the instance that buffers its receive path (the bridge UART).
// Cooperative, so it is not starved by the link sections, which loop on the
// main thread at priority 0 without yielding; below that, received frames would
// wait until such a loop happens to pause.
K_THREAD_STACK_DEFINE(serialRxStack, 4096);
static struct k_thread serialRxThread;

SerialLayer& SerialLayer::getInstance()
{
#if DT_NODE_EXISTS(DT_NODELABEL(cdc_acm_uart0))
    static SerialLayer instance(DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0)), Transport::Id::Serial, {});
#else
    static SerialLayer instance(nullptr, Transport::Id::Serial, {});
#endif
    return instance;
}

SerialLayer& SerialLayer::getBridgeInstance()
{
#if DT_HAS_CHOSEN(gblink_bridge_uart)
    static SerialLayer instance(DEVICE_DT_GET(DT_CHOSEN(gblink_bridge_uart)), Transport::Id::BridgeUart,
                                g_bridgeRxRing);
#else
    static SerialLayer instance(nullptr, Transport::Id::BridgeUart, g_bridgeRxRing);
#endif
    return instance;
}

SerialLayer::SerialLayer(const struct device* dev, Transport::Id id, std::span<uint8_t> rxRing)
    : m_target(dev), m_id(id), m_rxOnThread(!rxRing.empty())
{
    ring_buf_init(&m_txRing, sizeof(m_txRingMem), m_txRingMem);
    if (m_rxOnThread) ring_buf_init(&m_rxRing, rxRing.size(), rxRing.data());
    k_mutex_init(&m_txMutex);
    k_sem_init(&m_rxReady, 0, 1);
    initIfNeeded();
}

void SerialLayer::drainReceive()
{
    for (;;)
    {
        k_sem_take(&m_rxReady, K_FOREVER);
        uint8_t byte;
        while (ring_buf_get(&m_rxRing, &byte, 1) == 1) processIncomingByte(byte);
    }
}

void SerialLayer::initIfNeeded()
{
    if (m_ready) return;

    m_dev = m_target;
    if (m_dev == nullptr || !device_is_ready(m_dev)) {
        // Retry on next sendFrame() if the device wasn't bound yet at init.
        return;
    }

    uart_irq_callback_user_data_set(
        m_dev,
        [](const struct device* /*dev*/, void* ud) {
            static_cast<SerialLayer*>(ud)->onUartIrq();
        },
        this);

    if (m_rxOnThread) {
        k_thread_create(&serialRxThread, serialRxStack, K_THREAD_STACK_SIZEOF(serialRxStack),
                        [](void* self, void*, void*) { static_cast<SerialLayer*>(self)->drainReceive(); },
                        this, nullptr, nullptr, K_PRIO_COOP(CONFIG_NUM_COOP_PRIORITIES - 1), 0, K_NO_WAIT);
        k_thread_name_set(&serialRxThread, "serial_rx");
    }

    uart_irq_rx_enable(m_dev);
    m_ready = true;
}

void SerialLayer::onUartIrq()
{
    while (uart_irq_update(m_dev) && uart_irq_is_pending(m_dev))
    {
        if (uart_irq_rx_ready(m_dev))
        {
            uint8_t buf[64];
            int n = uart_fifo_read(m_dev, buf, sizeof(buf));
            if (!m_rxOnThread) {
                for (int i = 0; i < n; i++) processIncomingByte(buf[i]);
            } else if (n > 0) {
                if (ring_buf_put(&m_rxRing, buf, n) < static_cast<uint32_t>(n))
                    s_rxOverflows = s_rxOverflows + 1;
                k_sem_give(&m_rxReady);
            }
        }

        if (uart_irq_tx_ready(m_dev))
        {
            // ring_buf_get_claim/finish (rather than get + put-back) keeps
            // byte order intact if the UART FIFO can only accept part of
            // the offered chunk.
            uint8_t* tx_ptr;
            uint32_t claimed = ring_buf_get_claim(&m_txRing, &tx_ptr, 64);
            if (claimed == 0) {
                uart_irq_tx_disable(m_dev);
            } else {
                int written = uart_fifo_fill(m_dev, tx_ptr, claimed);
                ring_buf_get_finish(&m_txRing, written > 0 ? written : 0);
            }
        }
    }
}

void SerialLayer::processIncomingByte(uint8_t b)
{
    switch (m_rxState)
    {
        case RxState::sync1:
            if (b == syncByte0) m_rxState = RxState::sync2;
            break;
        case RxState::sync2:
            if (b == syncByte1) m_rxState = RxState::channel;
            else if (b == syncByte0) m_rxState = RxState::sync2;  // stay
            else m_rxState = RxState::sync1;
            break;
        case RxState::channel:
            m_rxChannel = b;
            m_rxState = RxState::lenLo;
            break;
        case RxState::lenLo:
            m_rxLen = b;
            m_rxState = RxState::lenHi;
            break;
        case RxState::lenHi:
            m_rxLen |= static_cast<uint16_t>(b) << 8;
            if (m_rxLen > maxPayload) {
                m_rxState = RxState::sync1;  // bad/oversized frame, resync
                break;
            }
            m_rxPos = 0;
            if (m_rxLen == 0) {
                dispatchFrame();
                m_rxState = RxState::sync1;
            } else {
                m_rxState = RxState::payload;
            }
            break;
        case RxState::payload:
            m_rxBuf[m_rxPos++] = b;
            if (m_rxPos >= m_rxLen) {
                dispatchFrame();
                m_rxState = RxState::sync1;
            }
            break;
    }
}

void SerialLayer::dispatchFrame()
{
    auto payload = std::span<const uint8_t>(m_rxBuf.data(), m_rxLen);

    // Only claim active transport when the frame matches a real channel + a
    // registered handler. Bytes from OS probing (e.g. ModemManager) can
    // coincidentally form a parseable header on an unknown channel; those are
    // dropped without flipping routing.
    if (m_rxChannel == channelCommand && m_commandHandler.handler != nullptr) {
        Transport::setActive(m_id);
        m_commandHandler.handler(payload, m_commandHandler.userData);
    } else if (m_rxChannel == channelData && m_dataHandler.handler != nullptr) {
        Transport::setActive(m_id);
        m_dataHandler.handler(payload, m_dataHandler.userData);
    }
}

bool SerialLayer::sendFrame(uint8_t channel, std::span<const uint8_t> payload)
{
    if (payload.size() > maxPayload) return false;
    initIfNeeded();
    if (!m_ready) return false;

    const uint8_t header[5] = {
        syncByte0,
        syncByte1,
        channel,
        static_cast<uint8_t>(payload.size() & 0xFF),
        static_cast<uint8_t>((payload.size() >> 8) & 0xFF)
    };

    const uint32_t total = sizeof(header) + payload.size();

    k_mutex_lock(&m_txMutex, K_FOREVER);

    if (ring_buf_space_get(&m_txRing) < total) {
        k_mutex_unlock(&m_txMutex);
        return false;
    }

    ring_buf_put(&m_txRing, header, sizeof(header));
    if (!payload.empty()) {
        ring_buf_put(&m_txRing, payload.data(), payload.size());
    }

    k_mutex_unlock(&m_txMutex);

    uart_irq_tx_enable(m_dev);
    return true;
}

bool SerialLayer::sendStatus(std::span<const uint8_t, 2> data)
{
    return sendFrame(channelStatus, data);
}

bool SerialLayer::sendData(std::span<const uint8_t> data)
{
    return sendFrame(channelData, data);
}

void SerialLayer::setReceiveCommandHandler(Transport::ReceiveHandler handler, void* userData)
{
    m_commandHandler = { handler, userData };
}

void SerialLayer::setReceiveDataHandler(Transport::ReceiveHandler handler, void* userData)
{
    m_dataHandler = { handler, userData };
}

// APPLICATION level so the USB stack (brought up at POST_KERNEL 2) is ready
// before the CDC-ACM UART is bound.
static int serial_layer_init(void)
{
    SerialLayer::getInstance();
    SerialLayer::getBridgeInstance();
    return 0;
}

SYS_INIT(serial_layer_init, APPLICATION, 0);
