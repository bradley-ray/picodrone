#include <stdio.h>
#include "btstack.h"
#include "btstack_event.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "mpu.h"
#include "control.h"
#include "bt.h"

#define RFCOMM_SERVER_CHANNEL 1
#define TIMER_PERIOD_MS 4

static int16_t throttle = 0;
static mpu_angle_t current_angles = {0};
static mpu_angle_t target_angles = {0};
static uint32_t start = 0;

static uint16_t rfcomm_channel_id;
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static uint8_t  spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static btstack_timer_source_t task_timer;
static uint8_t cmd_buf[16];
static void task_timer_handler(struct btstack_timer_source *ts) {
	mpu_update_accel();
	mpu_update_gyro();
	mpu_update_angles(&current_angles, time_us_32() - start);
	start = time_us_32();

	printf("p: %.2f, r: %.2f, y: %.2f\n\n", current_angles.pitch, current_angles.roll, current_angles.yaw);

	pid_step(throttle, &target_angles, &current_angles);

	// re-regiter timer
	btstack_run_loop_set_timer(ts, TIMER_PERIOD_MS);
	btstack_run_loop_add_timer(ts);
}

static void spp_service_setup(void){
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

    rfcomm_init();
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);

    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "Picodrone Flight Controller");
    sdp_register_service(spp_service_buffer);
    printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;

    bd_addr_t event_addr;
    uint8_t rfcomm_channel_nr;
    uint16_t mtu;
    int i;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case HCI_EVENT_PIN_CODE_REQUEST:
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    printf("SSP User Confirmation Request with numeric value '%d'\n", little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;
               
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
						// Turn on LED when connection succeeds
						cyw43_arch_gpio_put(0, 1);
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                    }
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
					// Turn off LED when connection closes
					cyw43_arch_gpio_put(0, 0);
                    printf("RFCOMM channel closed\n");
                    rfcomm_channel_id = 0;
                    break;
                
                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
			throttle = (int16_t)((packet[0] << 8) + packet[1]);
			target_angles.pitch = (int16_t) ((packet[2] << 8) + packet[3]);
			target_angles.roll = (int16_t)((packet[4] << 8) + packet[5]);
			target_angles.yaw = (int16_t)((packet[6] << 8) + packet[7]);
			printf("t: %d\n", throttle);
			printf("p: %f\n", target_angles.pitch);
			printf("r: %f\n", target_angles.roll);
			printf("y: %f\n\n", target_angles.yaw);
			break;
        default:
            break;
    }
}

int btstack_main(int argc, const char * argv[]) {
    (void)argc;
    (void)argv;

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

	task_timer.process = &task_timer_handler;
	btstack_run_loop_set_timer(&task_timer, TIMER_PERIOD_MS);
	btstack_run_loop_add_timer(&task_timer);

    spp_service_setup();

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("Picodrone Flight Controller 00:00:00:00:00:00");

    hci_power_control(HCI_POWER_ON);

    return 0;
}
