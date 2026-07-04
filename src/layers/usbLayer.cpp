#include "usbLayer.hpp"
#include "../persist.hpp"

#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/bos.h>
#include <zephyr/usb/msos_desc.h>
#include <zephyr/kernel.h>

namespace
{
    bool g_enabled = false;
    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    /* random GUID {FA611CC3-7057-42EE-9D82-4919639562B3} */
    #define WEBUSB_DEVICE_INTERFACE_GUID \
        '{', 0x00, 'F', 0x00, 'A', 0x00, '6', 0x00, '1', 0x00, '1', 0x00, \
        'C', 0x00, 'C', 0x00, '3', 0x00, '-', 0x00, '7', 0x00, '0', 0x00, \
        '5', 0x00, '7', 0x00, '-', 0x00, '4', 0x00, '2', 0x00, 'E', 0x00, \
        'E', 0x00, '-', 0x00, '9', 0x00, 'D', 0x00, '8', 0x00, '2', 0x00, \
        '-', 0x00, '4', 0x00, '9', 0x00, '1', 0x00, '9', 0x00, '6', 0x00, \
        '3', 0x00, '9', 0x00, '5', 0x00, '6', 0x00, '2', 0x00, 'B', 0x00, \
        '3', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00

    #define COMPATIBLE_ID_WINUSB \
        'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00

    static struct msosv2_descriptor_t {
        struct msosv2_descriptor_set_header header;
        struct msosv2_compatible_id webusb_compatible_id;
        struct msosv2_guids_property webusb_guids_property;
    } __packed msosv2_descriptor = {
        .header = {
            .wLength = sizeof(struct msosv2_descriptor_set_header),
            .wDescriptorType = MS_OS_20_SET_HEADER_DESCRIPTOR,
            .dwWindowsVersion = 0x06030000,
            .wTotalLength = sizeof(struct msosv2_descriptor_t),
        },
        .webusb_compatible_id = {
            .wLength = sizeof(struct msosv2_compatible_id),
            .wDescriptorType = MS_OS_20_FEATURE_COMPATIBLE_ID,
            .CompatibleID = {COMPATIBLE_ID_WINUSB},
        },
        .webusb_guids_property = {
            .wLength = sizeof(struct msosv2_guids_property),
            .wDescriptorType = MS_OS_20_FEATURE_REG_PROPERTY,
            .wPropertyDataType = MS_OS_20_PROPERTY_DATA_REG_MULTI_SZ,
            .wPropertyNameLength = 42,
            .PropertyName = {DEVICE_INTERFACE_GUIDS_PROPERTY_NAME},
            .wPropertyDataLength = 80,
            .bPropertyData = {WEBUSB_DEVICE_INTERFACE_GUID},
        }
    };

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
    // BOS Descriptor
    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    USB_DEVICE_BOS_DESC_DEFINE_CAP struct usb_bos_webusb_desc {
        struct usb_bos_platform_descriptor platform;
        struct usb_bos_capability_webusb cap;
    } __packed bos_cap_webusb = {
        /* WebUSB Platform Capability Descriptor:
        * https://wicg.github.io/webusb/#webusb-platform-capability-descriptor
        */
        .platform = {
            .bLength = sizeof(struct usb_bos_platform_descriptor)
                + sizeof(struct usb_bos_capability_webusb),
            .bDescriptorType = USB_DESC_DEVICE_CAPABILITY,
            .bDevCapabilityType = USB_BOS_CAPABILITY_PLATFORM,
            .bReserved = 0,
            /* WebUSB Platform Capability UUID
            * 3408b638-09a9-47a0-8bfd-a0768815b665
            */
            .PlatformCapabilityUUID = {
                0x38, 0xB6, 0x08, 0x34,
                0xA9, 0x09,
                0xA0, 0x47,
                0x8B, 0xFD,
                0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
            },
        },
        .cap = {
            .bcdVersion = sys_cpu_to_le16(0x0100),
            .bVendorCode = 0x01,
            .iLandingPage = 0x01
        }
    };

    USB_DEVICE_BOS_DESC_DEFINE_CAP struct usb_bos_msosv2_desc {
        struct usb_bos_platform_descriptor platform;
        struct usb_bos_capability_msos cap;
    } __packed bos_cap_msosv2 = {
        /* Microsoft OS 2.0 Platform Capability Descriptor
        * See https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/
        * microsoft-defined-usb-descriptors
        * Adapted from the source:
        * https://github.com/sowbug/weblight/blob/master/firmware/webusb.c
        * (BSD-2) Thanks http://janaxelson.com/files/ms_os_20_descriptors.c
        */
        .platform = {
            .bLength = sizeof(struct usb_bos_platform_descriptor)
                + sizeof(struct usb_bos_capability_msos),
            .bDescriptorType = USB_DESC_DEVICE_CAPABILITY,
            .bDevCapabilityType = USB_BOS_CAPABILITY_PLATFORM,
            .bReserved = 0,
            .PlatformCapabilityUUID = {
                /**
                 * MS OS 2.0 Platform Capability ID
                 * D8DD60DF-4589-4CC7-9CD2-659D9E648A9F
                 */
                0xDF, 0x60, 0xDD, 0xD8,
                0x89, 0x45,
                0xC7, 0x4C,
                0x9C, 0xD2,
                0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
            },
        },
        .cap = {
            /* Windows version (8.1) (0x06030000) */
            .dwWindowsVersion = sys_cpu_to_le32(0x06030000),
            .wMSOSDescriptorSetTotalLength =
                sys_cpu_to_le16(sizeof(msosv2_descriptor)),
            /* Arbitrary code that is used as bRequest for vendor command */
            .bMS_VendorCode = 0x02,
            .bAltEnumCode = 0x00
        },
    };

    USB_DEVICE_BOS_DESC_DEFINE_CAP struct usb_bos_capability_lpm bos_cap_lpm = {
        .bLength = sizeof(struct usb_bos_capability_lpm),
        .bDescriptorType = USB_DESC_DEVICE_CAPABILITY,
        .bDevCapabilityType = USB_BOS_CAPABILITY_EXTENSION,
        /**
         * Currently there is not a single device driver in Zephyr that supports
         * LPM. Moreover, Zephyr USB stack does not have LPM support, so do not
         * falsely claim to support LPM.
         * BIT(1) - LPM support
         * BIT(2) - BESL support
         */
        .bmAttributes = 0,
    };

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    /* WebUSB Device Requests */
    static const uint8_t webusb_allowed_origins[] = {
        /* Allowed Origins Header:
        * https://wicg.github.io/webusb/#get-allowed-origins
        */
        0x05, 0x00, 0x0D, 0x00, 0x01,

        /* Configuration Subset Header:
        * https://wicg.github.io/webusb/#configuration-subset-header
        */
        0x04, 0x01, 0x01, 0x01,

        /* Function Subset Header:
        * https://wicg.github.io/webusb/#function-subset-header
        */
        0x04, 0x02, 0x02, 0x01
    };

    /* Number of allowed origins */
    #define NUMBER_OF_ALLOWED_ORIGINS   1

    /* URL Descriptor: https://wicg.github.io/webusb/#url-descriptor
     * Chrome only surfaces the landing-page notification for https URLs, so the
     * scheme must be 1 (https). bLength = 3 (header) + strlen("launcher.gblink.io"). */
    static const uint8_t webusb_origin_url[] = {
        /* Length, DescriptorType, Scheme (1 = https) */
        0x15, 0x03, 0x01,
        'l', 'a', 'u', 'n', 'c', 'h', 'e', 'r', '.', 'g', 'b', 'l', 'i', 'n', 'k', '.', 'i', 'o'
    };

    /* Predefined response to control commands related to MS OS 1.0 descriptors
    * Please note that this code only defines "extended compat ID OS feature
    * descriptors" and not "extended properties OS features descriptors"
    */
    #define MSOS_STRING_LENGTH	18
    static struct string_desc {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bString[MSOS_STRING_LENGTH];

    } __packed msos1_string_descriptor = {
        .bLength = MSOS_STRING_LENGTH,
        .bDescriptorType = USB_DESC_STRING,
        /* Signature MSFT100 */
        .bString = {
            'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00,
            '1', 0x00, '0', 0x00, '0', 0x00,
            0x03, /* Vendor Code, used for a control request */
            0x00, /* Padding byte for VendorCode looks like UTF16 */
        },
    };

    static const uint8_t msos1_compatid_descriptor[] = {
        /* See https://github.com/pbatard/libwdi/wiki/WCID-Devices */
        /* MS OS 1.0 header section */
        0x28, 0x00, 0x00, 0x00, /* Descriptor size (40 bytes)          */
        0x00, 0x01,             /* Version 1.00                        */
        0x04, 0x00,             /* Type: Extended compat ID descriptor */
        0x01,                   /* Number of function sections         */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       /* reserved    */

        /* MS OS 1.0 function section */
        0x02,     /* Index of interface this section applies to. */
        0x01,     /* reserved */
        /* 8-byte compatible ID string, then 8-byte sub-compatible ID string */
        'W',  'I',  'N',  'U',  'S',  'B',  0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00 /* reserved */
    };

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
    // Handler
    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    int customRequestHandler(struct usb_setup_packet *pSetup,
                int32_t *len, uint8_t **data)
    {
        if (usb_reqtype_is_to_device(pSetup)) {
            return -ENOTSUP;
        }

        if (USB_GET_DESCRIPTOR_TYPE(pSetup->wValue) == USB_DESC_STRING &&
            USB_GET_DESCRIPTOR_INDEX(pSetup->wValue) == 0xEE) {
            *data = (uint8_t *)(&msos1_string_descriptor);
            *len = sizeof(msos1_string_descriptor);
            return 0;
        }

        return -EINVAL;
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    int vendorRequestHandler(struct usb_setup_packet *pSetup,
                int32_t *len, uint8_t **data)
    {
        if (usb_reqtype_is_to_device(pSetup)) {
            return -ENOTSUP;
        }

        /* Get Allowed origins request */
        if (pSetup->bRequest == 0x01 && pSetup->wIndex == 0x01) {
            *data = (uint8_t *)(&webusb_allowed_origins);
            *len = sizeof(webusb_allowed_origins);
            return 0;
        } else if (pSetup->bRequest == 0x01 && pSetup->wIndex == 0x02) {
            /* Get URL request */
            uint8_t index = USB_GET_DESCRIPTOR_INDEX(pSetup->wValue);

            if (index == 0U || index > NUMBER_OF_ALLOWED_ORIGINS) {
                return -ENOTSUP;
            }

            *data = (uint8_t *)(&webusb_origin_url);
            *len = sizeof(webusb_origin_url);

            return 0;
        } else if (pSetup->bRequest == bos_cap_msosv2.cap.bMS_VendorCode &&
            pSetup->wIndex == MS_OS_20_DESCRIPTOR_INDEX) {
            /* Get MS OS 2.0 Descriptors request */
            *data = (uint8_t *)(&msosv2_descriptor);
            *len = sizeof(msosv2_descriptor);

            return 0;
        } else if (pSetup->bRequest == 0x03 && pSetup->wIndex == 0x04) {
            /* Get MS OS 1.0 Descriptors request */
            /* 0x04 means "Extended compat ID".
            * Use 0x05 instead for "Extended properties".
            */
            *data = (uint8_t *)(&msos1_compatid_descriptor);
            *len = sizeof(msos1_compatid_descriptor);

            return 0;
        }

        return -ENOTSUP;
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    struct UsbConfig 
    {
        struct usb_if_descriptor if0;
        struct usb_ep_descriptor if0_in_ep_data;
        struct usb_ep_descriptor if0_out_ep_data;
        struct usb_ep_descriptor if0_in_ep_command;
        struct usb_ep_descriptor if0_out_ep_command;
    } __packed;

    USBD_CLASS_DESCR_DEFINE(primary, 0) UsbConfig m_usbServiceDescriptor = {
        .if0 = {
            .bLength = sizeof(struct usb_if_descriptor),
            .bDescriptorType = USB_DESC_INTERFACE,
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 4,
            .bInterfaceClass = USB_BCC_VENDOR,
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 0,
            .iInterface = 0
        },
        // Bulk endpoints — RP2040's legacy USB stack misbehaves with mixed
        // interrupt + bulk in a composite (vendor + CDC) configuration.
        .if0_in_ep_data = {
            .bLength = sizeof(struct usb_ep_descriptor),
            .bDescriptorType = USB_DESC_ENDPOINT,
            .bEndpointAddress = AUTO_EP_IN | 0x1,
            .bmAttributes = USB_DC_EP_BULK,
            .wMaxPacketSize = sys_cpu_to_le16(UsbLayer::endpointSize()),
            .bInterval = 0
        },
        .if0_out_ep_data = {
            .bLength = sizeof(struct usb_ep_descriptor),
            .bDescriptorType = USB_DESC_ENDPOINT,
            .bEndpointAddress = AUTO_EP_OUT | 0x1,
            .bmAttributes = USB_DC_EP_BULK,
            .wMaxPacketSize = sys_cpu_to_le16(UsbLayer::endpointSize()),
            .bInterval = 0
        },
        .if0_in_ep_command = {
            .bLength = sizeof(struct usb_ep_descriptor),
            .bDescriptorType = USB_DESC_ENDPOINT,
            .bEndpointAddress = AUTO_EP_IN | 0x2,
            .bmAttributes = USB_DC_EP_BULK,
            .wMaxPacketSize = sys_cpu_to_le16(UsbLayer::endpointSize()),
            .bInterval = 0
        },
        .if0_out_ep_command = {
            .bLength = sizeof(struct usb_ep_descriptor),
            .bDescriptorType = USB_DESC_ENDPOINT,
            .bEndpointAddress = AUTO_EP_OUT | 0x2,
            .bmAttributes = USB_DC_EP_BULK,
            .wMaxPacketSize = sys_cpu_to_le16(UsbLayer::endpointSize()),
            .bInterval = 0
        }
    };

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    struct usb_ep_cfg_data m_webUsbEndpointConfig[] = {
        {
            .ep_cb = usb_transfer_ep_callback,
            .ep_addr = AUTO_EP_IN | 0x1
        },
        {
            .ep_cb	= usb_transfer_ep_callback,
            .ep_addr = AUTO_EP_OUT | 0x1
        },
        {
            .ep_cb = usb_transfer_ep_callback,
            .ep_addr = AUTO_EP_IN | 0x2
        },
        {
            .ep_cb	= usb_transfer_ep_callback,
            .ep_addr = AUTO_EP_OUT | 0x2
        }
    };

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    static void usbStatusCallback(struct usb_cfg_data *cfg,
				 enum usb_dc_status_code status,
				 const uint8_t *param)
    {
        ARG_UNUSED(param);
        ARG_UNUSED(cfg);
        if (g_enabled)
        {
            UsbLayer::getInstance().setStatus(status);
        }
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    USBD_DEFINE_CFG_DATA(m_usbConfig) = {
        .usb_device_description = NULL,
        .interface_descriptor = &m_usbServiceDescriptor.if0,
        .cb_usb_status = usbStatusCallback,
        .interface = {
            .class_handler = NULL,
            .vendor_handler = vendorRequestHandler,
            .custom_handler = customRequestHandler,
        },
        .num_endpoints = ARRAY_SIZE(m_webUsbEndpointConfig),
        .endpoint = m_webUsbEndpointConfig
    };

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    static int usb_init()
    {
        // Advertise the WebUSB landing page only if the user hasn't turned it
        // off. The host reads this at enumeration, so the toggle takes effect on
        // the next reconnect.
        bos_cap_webusb.cap.iLandingPage = landingPageEnabled() ? 0x01 : 0x00;

        usb_bos_register_cap((void *)&bos_cap_webusb);
	    usb_bos_register_cap((void *)&bos_cap_msosv2);
	    usb_bos_register_cap((void *)&bos_cap_lpm);

        int ret = usb_enable(NULL);
        if (ret == 0) g_enabled = true;
        return ret;
    }
}

SYS_INIT(usb_init, POST_KERNEL, 2);