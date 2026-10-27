/* Negative priority is cooperative
   Non-negative priority is preemptible (such as during any k_sleep)
   Higher value is lower priority
*/
#define CONSOLE_THREAD_PRIORITY 6
#define HID_DROPPED_REPORTS_LOGGING_PRIORITY 6
#define USB_INIT_THREAD_PRIORITY 6
#define ESB_THREAD_PRIORITY 6
#define LED_THREAD_PRIORITY 6
#define STATUS_THREAD_PRIORITY 6
