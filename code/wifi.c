#include "wifi.h"

uint8 image_copy[MT9V03X_H][MT9V03X_W];

void wifi_init(void)
{
    while (wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);
    }

    if (0 == WIFI_SPI_AUTO_CONNECT)
    {
        while (wifi_spi_socket_connect(
            "TCP",
            TCP_TARGET_IP,
            TCP_TARGET_PORT,
            WIFI_LOCAL_PORT))
        {
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);
        }
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
}

void wifi_send_data(void)
{
    seekfree_assistant_oscilloscope_struct oscilloscope_data;

    oscilloscope_data.data[0] = current_state;
    
    oscilloscope_data.channel_num = 1;
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
}

void wifi_send_image(void)
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

        seekfree_assistant_camera_send();
    }
}
