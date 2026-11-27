
#include <cerrno>
#include <cstdint>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <usb_descriptor.h>
#include <span>
#include <array>
#include <algorithm>
#include <zephyr/kernel.h>
#include "../hardware.hpp"
#include "../persist.hpp"
#include "transport.hpp"

#pragma once

class UsbLayer
{
    static constexpr uint8_t commandOutEndpoint = 1;
    static constexpr uint8_t statusInEndpoint = 129;
    static constexpr uint8_t dataOutEndpoint = 2;
    static constexpr uint8_t dataInEndpoint = 130;

    static constexpr uint16_t m_endpointSize = 64;


public:
    using UsbReceiveHandler = void(*)(std::span<const uint8_t>, void*);

    static UsbLayer& getInstance() 
    {
        static UsbLayer instance = UsbLayer();
        return instance;
    }

    UsbLayer(const UsbLayer&) = delete;
    UsbLayer& operator=(const UsbLayer&) = delete;
    UsbLayer(UsbLayer&&) = delete;
    UsbLayer& operator=(UsbLayer&&) = delete;

    bool sendStatus(std::span<const uint8_t, 2> data)
    {
        if (!m_endpointsEnabled) return false;

        // Wait for any previous status transfer to complete before staging a new
        // one.  Without this, back-to-back status messages (e.g. DeviceReady
        // followed immediately by AwaitMode) race on the same IN endpoint and the
        // second transfer is silently dropped — breaking GBA online mode.
        // Timeout of 100 ms avoids blocking forever if the host stops reading.
        k_sem_take(&m_statusTransferDone, K_MSEC(100));

        std::ranges::copy(data, m_sendStatusData.begin());
        int ret = usb_transfer(statusInEndpoint, m_sendStatusData.data(), data.size_bytes(),
            USB_TRANS_WRITE, m_usbWriteStatusCallback, this);

        if (ret != 0) {
            // Transfer failed to start — release the semaphore so the next call
            // isn't stuck waiting for a callback that will never fire.
            k_sem_give(&m_statusTransferDone);
            return false;
        }
        return true;
    }

    bool sendData(std::span<const uint8_t> data)
    {
        if (!m_endpointsEnabled) return false;

        // Same race as sendStatus: back-to-back data sends (e.g. a >64-byte
        // protocol frame split across transport chunks) would overwrite
        // m_sendData while the previous async transfer is still reading it.
        k_sem_take(&m_dataTransferDone, K_MSEC(100));

        return startDataTransfer(data);
    }

    // Like sendData, but fails at once while the host has not taken the previous
    // transfer yet, instead of waiting. For senders that keep their own queue.
    bool trySendData(std::span<const uint8_t> data)
    {
        if (!m_endpointsEnabled) return false;
        if (k_sem_take(&m_dataTransferDone, K_NO_WAIT) != 0) return false;

        return startDataTransfer(data);
    }

    void setReceiveCommandHandler(const UsbReceiveHandler& handler, void* userData)
    {
        m_receiveCommandHandler.userData = userData;
        m_receiveCommandHandler.handler = handler;
    }

    void setReceiveDataHandler(const UsbReceiveHandler& handler, void* userData)
    {
        m_receiveDataCommandHandler.handler = handler;
        m_receiveDataCommandHandler.userData = userData;
    }

    static constexpr uint16_t endpointSize() { return m_endpointSize; }

    void setStatus(enum usb_dc_status_code status)
    {
        switch (status) 
        {
        case USB_DC_CONFIGURED:
            m_endpointsEnabled = true;
            applyLedForSlot(LED_SLOT_IDLE); // connected, no active mode (user-configurable)
            break;
        case USB_DC_ERROR: 
            [[fallthrough]];
        case USB_DC_DISCONNECTED: 
            [[fallthrough]];
        case USB_DC_RESET: 
            [[fallthrough]];
        case USB_DC_UNKNOWN:
            m_endpointsEnabled = false;
            Hardware::getInstance().setLED(5, 0, 0, true); // Red = power on, no USB
            break;

        case USB_DC_CONNECTED:
        case USB_DC_SUSPEND:
        case USB_DC_RESUME:
            break;
        case USB_DC_CLEAR_HALT:
            //sys_reboot(SYS_REBOOT_WARM);
        default:
            break;
        }
    }

private:
    bool m_endpointsEnabled = false;
    struct k_sem m_waitForFreeEndpoint;

    struct ReceiveDelegate
    {
        void* userData;
        UsbReceiveHandler handler;
        uint8_t endpoint;
        UsbLayer* layer;
        std::array<uint8_t, m_endpointSize> endpointBuffer = {};

        void operator()(std::span<const uint8_t> data)
        {
            if (handler != nullptr)
            {
                handler(data, userData);
            }
        }
    };

    struct k_msgq m_statusMsgQueue;
    std::array<char, 100> m_statusQueueBuffer;

    int perpareNextReceive(ReceiveDelegate& delegate)
    {
        delegate.endpointBuffer.fill(0);
        return usb_transfer(delegate.endpoint, delegate.endpointBuffer.data(), delegate.endpointBuffer.size(), USB_TRANS_READ, m_usbReadCallback, &delegate);
    }

    void receive(int size, ReceiveDelegate* delegate)
    {
        if (size > 0)
        {
            Transport::setActive(Transport::Id::Usb);
            (*delegate)(std::span(delegate->endpointBuffer.begin(), static_cast<size_t>(size)));
        }
        perpareNextReceive(*delegate);
    };

    bool startDataTransfer(std::span<const uint8_t> data)
    {
        std::ranges::copy(data, m_sendData.begin());
        // usb_transfer is async (queues a k_work item), but with bInterval=1 on
        // the data endpoint the host polls every 1 ms, so the work-queue delay
        // is absorbed within the next poll cycle and has no meaningful impact.
        int ret = usb_transfer(dataInEndpoint, m_sendData.data(), data.size_bytes(),
            USB_TRANS_WRITE, m_usbWriteDataCallback, this);

        if (ret != 0) {
            k_sem_give(&m_dataTransferDone);
            return false;
        }
        return true;
    }

    ReceiveDelegate m_receiveCommandHandler = {
        .userData = nullptr,
        .handler = nullptr,
        .endpoint = commandOutEndpoint,
        .layer = this
    };

    ReceiveDelegate m_receiveDataCommandHandler = {
        .userData = nullptr,
        .handler = nullptr,
        .endpoint = dataOutEndpoint,
        .layer = this
    };
    
    std::array<uint8_t, m_endpointSize> m_sendData = {};
    std::array<uint8_t, 2> m_sendStatusData = {};  // Separate buffer for status to avoid race
    struct k_sem m_statusTransferDone;
    struct k_sem m_dataTransferDone;

    UsbLayer()
    {
        perpareNextReceive(m_receiveCommandHandler);
        perpareNextReceive(m_receiveDataCommandHandler);
        k_sem_init(&m_waitForFreeEndpoint, 1, 1);
        k_sem_init(&m_statusTransferDone, 1, 1);  // starts at 1 so first sendStatus can proceed
        k_sem_init(&m_dataTransferDone, 1, 1);
        k_msgq_init(&m_statusMsgQueue, m_statusQueueBuffer.data(), 2, m_statusQueueBuffer.size() / 2);
    }

    static void m_usbReadCallback(uint8_t ep, int size, void* userData)
    {
        ReceiveDelegate* self = static_cast<ReceiveDelegate*>(userData);
        self->layer->receive(size, self);
    }

    static void m_usbWriteDataCallback(uint8_t ep, int size, void* userData)
    {
        UsbLayer* self = static_cast<UsbLayer*>(userData);
        k_sem_give(&self->m_dataTransferDone);
    }

    static void m_usbWriteStatusCallback(uint8_t ep, int size, void* userData)
    {
        UsbLayer* self = static_cast<UsbLayer*>(userData);
        k_sem_give(&self->m_statusTransferDone);
    }
};
