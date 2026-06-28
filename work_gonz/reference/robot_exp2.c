#include <libpynq.h>
#include <stepper.h>

#if defined(DISABLE_VL53L0X)
#define HAVE_VL53L0X 0
typedef struct {
  int unavailable;
} vl53x;
#elif defined(__has_include)
#if __has_include("vl53l0x.h")
#define HAVE_VL53L0X 1
#include <iic.h>
#include "vl53l0x.h"
#else
#define HAVE_VL53L0X 0
typedef struct {
  int unavailable;
} vl53x;
#endif
#else
#define HAVE_VL53L0X 1
#include <iic.h>
#include "vl53l0x.h"
#endif

#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef MODULE_NUMBER
#define MODULE_NUMBER "74"
#endif

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0)
#endif

#define UART_RX_PIN IO_AR0
#define UART_TX_PIN IO_AR1

#define LEFT_IR_PIN IO_AR13
#define RIGHT_IR_PIN IO_AR12
#define BLACK_DETECTED_LEVEL GPIO_LEVEL_HIGH

#define COLOR_S0_PIN IO_AR5
#define COLOR_S1_PIN IO_AR4
#define COLOR_S2_PIN IO_AR8
#define COLOR_S3_PIN IO_AR7
#define COLOR_OUT_PIN IO_AR6

#define ULTRASONIC_TRIG_PIN IO_AR9
#define ULTRASONIC_ECHO_PIN IO_AR10

#define POLL_DELAY_MS 10U
#define STATUS_PERIOD_MS 1000U

#define UART_PAYLOAD_MAX 512U
#define MQTT_JSON_MAX 512U

#define SEARCH_STEP_M 0.04
#define MAP_STEP_M 0.04
#define LINE_CLEARANCE_STEP_M 0.03
#define IR_SENSOR_FORWARD_OFFSET_M 0.06
#define ALIGN_STEP_RAD (2.0 * M_PI / 180.0)
#define TURN_TO_LINE_RAD (90.0 * M_PI / 180.0)
#define WALL_OVERTURN_RAD (3.0 * M_PI / 180.0) //from 4.0
#define MAX_ALIGN_STEPS 120U
#define MAX_MAP_STEPS 180U
#define MAP_CORNER_GRACE_STEPS 3U
#define LOOP_CLOSE_RADIUS_M 0.12
#define LOOP_REVISIT_RADIUS_M 0.10
#define LOOP_MIN_CORNERS 4U
#define LOOP_REVISIT_MIN_CORNERS 3U
#define LOOP_MIN_DISTANCE_M 1.0
#define LOOP_REVISIT_IGNORE_RECENT_SAMPLES 8U
#define ANTICLOCKWISE_FOLLOW_TURN_SIGN 1

#define STEPS_PER_REV 200.0
#define MICROSTEP_FACTOR 8.0
#define WHEEL_DIAMETER_M 0.08
#define WHEEL_BASE_M 0.124
#define STEP_DISTANCE_SCALE 1.03
#define TURN_STEP_SCALE 1.0
#define SCAN_STEP_RAD (10.0 * M_PI / 180.0)
#define SCAN_SETTLE_MS 250U
#define TOF_MAX_MOUNTAIN_DISTANCE_M 1.5
#define TOF_ZERO_OFFSET_MM 23U
#define VL53L0X_ADDR 0x29
#define RANGE_MODE_LONG 0

#define FILTER_SETTLE_MS 20U
#define SAMPLE_WINDOW_MS 100U
#define COLOR_SAMPLE_DURATION_MS 3000U
#define COLOR_TRIGGER_DISTANCE_M 0.50
#define ULTRASONIC_DOWN_ANGLE_DEG 3.0
#define ULTRASONIC_HEIGHT_M 0.05
#define COLOR_SENSOR_FORWARD_OFFSET_M 0.01
#define COLOR_APPROACH_MIN_DISTANCE_M 0.01
#define COLOR_APPROACH_MAX_DISTANCE_M 0.45
#define COLOR_ULTRASONIC_POLL_MS 150U
#define TRIGGER_PULSE_US 10
#define ECHO_START_TIMEOUT_US 30000
#define ECHO_END_TIMEOUT_US 30000
#define SOUND_SPEED_M_PER_S 343.0

#define COLOR_MODEL_FEATURES 4U
#define COLOR_MODEL_LABELS 5U
#define COLOR_MODEL_DIST_FILL_CM 13.996250000000003

static const double COLOR_MODEL_MEAN[COLOR_MODEL_FEATURES] = {
    1503.5, 1185.125, 1424.75, COLOR_MODEL_DIST_FILL_CM};
static const double COLOR_MODEL_STD[COLOR_MODEL_FEATURES] = {
    774.5042769410637, 811.9635055684461, 973.8924157729127,
    7.888464580480792};
static const double COLOR_MODEL_W[COLOR_MODEL_LABELS][COLOR_MODEL_FEATURES] = {
    {2.3379380989148553, -1.5368433654638065, -1.1166018601671737,
     0.068658012903017},
    {-0.24131429756249878, 2.9822676698120807, -2.7571751080973903,
     0.0711898066422565},
    {-1.704690243896113, -1.6395073390946766, 3.490343737233205,
     -0.21862241755161974},
    {0.904494512874087, 0.6025213212953023, 0.9078674486592412,
     0.10528300121068215},
    {-1.5668959647656575, -0.8023347103380889, -0.5922818756767573,
     -0.14852811519970405},
};
static const double COLOR_MODEL_B[COLOR_MODEL_LABELS] = {
    -0.9951062988743739, -1.1790727413011182, -1.3616654532103518,
    -0.6885006303103918, -2.149780803010174};

#define SEARCH_PULSE_DELAY_TICKS 50000U
#define MAP_PULSE_DELAY_TICKS 65000U
#define CLEARANCE_PULSE_DELAY_TICKS 65000U
#define ALIGN_PULSE_DELAY_TICKS 65000U
#define TURN_PULSE_DELAY_TICKS 65000U
#define SCAN_PULSE_DELAY_TICKS 60000U
#define COLOR_APPROACH_PULSE_DELAY_TICKS 65500U
#define MOTOR_ENABLE_SETTLE_MS 250U
#define PRE_COMMAND_SETTLE_MS 20U
#define POST_COMMAND_SETTLE_MS 20U

#define AREA_DEFAULT_MIN_X_M (-0.75)
#define AREA_DEFAULT_MAX_X_M 0.75
#define AREA_DEFAULT_MIN_Y_M (-0.75)
#define AREA_DEFAULT_MAX_Y_M 0.75
#define AREA_MARGIN_M 0.08
#define AREA_SWEEP_STEP_M 0.05
#define AREA_ROW_SPACING_M 0.12
#define AREA_BOUNDARY_EPS_M 0.03
#define AREA_BORDER_CLOSE_M 0.14
#define AREA_OBJECT_AVOID_DISTANCE_M 0.35
#define AREA_OBJECT_AVOID_BEARING_RAD (30.0 * M_PI / 180.0)
#define AREA_AVOID_BACKUP_M (-0.06)
#define AREA_AVOID_FORWARD_M 0.08
#define AREA_AVOID_TURN_RAD (35.0 * M_PI / 180.0)
#define AREA_MAX_BORDER_SAMPLES 256U
#define MAP_HINT_MAX_AGE_MS 3000U

#define TIMER_TICKS_PER_SECOND 100000000.0
#define NOMINAL_STEP_DISTANCE_M                                                \
  ((M_PI * WHEEL_DIAMETER_M) / (STEPS_PER_REV * MICROSTEP_FACTOR))
#define MAX_STEPPER_STEPS 32767
#define MIN_STEPPER_STEPS (-32768)

typedef struct {
  char payload[UART_PAYLOAD_MAX + 1U];
  uint8_t length_bytes[sizeof(uint32_t)];
  size_t length_index;
  uint32_t payload_length;
  uint32_t payload_index;
  bool discarding_payload;
} uart_rx_t;

typedef struct {
  double coord_x;
  double coord_y;
  double theta;
} telemetry_state_t;

typedef struct {
  bool left_black;
  bool right_black;
} ir_readings_t;

typedef enum {
  DETECT_RED = 0,
  DETECT_GREEN,
  DETECT_BLUE,
  DETECT_WHITE,
  DETECT_BLACK
} color_label_t;

typedef struct {
  uint32_t red;
  uint32_t green;
  uint32_t blue;
} color_sample_t;

typedef enum {
  MODE_IDLE = 0,
  MODE_MOUNTAIN_DETECT,
  MODE_BORDER_SEARCH,
  MODE_ALIGN_LINE,
  MODE_TURN_AWAY_FROM_LINE,
  MODE_LEAVE_LINE,
  MODE_TURN_TO_LINE,
  MODE_BORDER_MAP,
  MODE_COLOR_APPROACH,
  MODE_COLOR_SAMPLE,
  MODE_AREA_SWEEP,
  MODE_AREA_AVOID
} control_mode_t;

typedef enum {
  AREA_PHASE_ALIGN = 0,
  AREA_PHASE_SWEEP,
  AREA_PHASE_TURN_TO_ROW,
  AREA_PHASE_ROW_SHIFT,
  AREA_PHASE_TURN_TO_SWEEP,
  AREA_PHASE_AVOID_BACKUP,
  AREA_PHASE_AVOID_TURN,
  AREA_PHASE_AVOID_FORWARD,
  AREA_PHASE_DONE
} area_phase_t;

typedef struct {
  bool active;
  bool telemetry_now;
  bool scan_waiting_to_read;
  uint16_t scan_step_index;
  uint16_t scan_step_count;
  uint64_t started_at_ms;
  uint64_t estimated_duration_ms;
  uint64_t scan_read_at_ms;
  double start_x;
  double start_y;
  double start_theta;
  double target_x;
  double target_y;
  double target_theta;
} motion_state_t;

typedef struct {
  int align_turn_sign;
  int follow_turn_sign;
  bool first_left_black;
  bool first_right_black;
  bool loop_start_set;
  uint16_t align_steps;
  uint16_t map_steps;
  uint16_t corner_count;
  double align_start_theta;
  double loop_start_x;
  double loop_start_y;
  double line_anchor_x;
  double line_anchor_y;
  double line_normal_theta;
  double line_theta;
  double follow_theta;
  double line_distance_m;
  double total_mapped_distance_m;
  double sample_x[AREA_MAX_BORDER_SAMPLES];
  double sample_y[AREA_MAX_BORDER_SAMPLES];
  uint16_t sample_count;
} border_state_t;

typedef struct {
  uint64_t next_ultrasonic_poll_ms;
  double trigger_distance_m;
  double target_x;
  double target_y;
  control_mode_t resume_mode;
} color_object_state_t;

typedef struct {
  bool active;
  area_phase_t phase;
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double row_spacing_m;
  double sweep_step_m;
  double sweep_theta;
  int sweep_sign;
  int row_sign;
} area_state_t;

typedef struct {
  bool valid;
  uint64_t received_at_ms;
  double obstacle_distance_m;
  double obstacle_bearing_rad;
  double nearest_border_distance_m;
} map_hint_state_t;

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signum) {
  (void)signum;
  keep_running = 0;
}

static uint64_t monotonic_ms(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    return 0U;
  }
  return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

static uint64_t monotonic_usec(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    return 0U;
  }
  return ((uint64_t)ts.tv_sec * 1000000ULL) + ((uint64_t)ts.tv_nsec / 1000ULL);
}

static void sleep_usec_exact(long usec) {
  struct timespec req;

  req.tv_sec = usec / 1000000L;
  req.tv_nsec = (usec % 1000000L) * 1000L;
  nanosleep(&req, NULL);
}

static double clamp01(double value) {
  if (value < 0.0) {
    return 0.0;
  }
  if (value > 1.0) {
    return 1.0;
  }
  return value;
}

static double normalize_angle(double angle_rad) {
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad <= -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

static double angle_midpoint(double start_rad, double end_rad) {
  return start_rad + (0.5 * normalize_angle(end_rad - start_rad));
}

static double shortest_delta(double from_rad, double to_rad) {
  return normalize_angle(to_rad - from_rad);
}

static double ultrasonic_ground_projection_m(double slant_distance_m) {
  const double down_angle_rad = ULTRASONIC_DOWN_ANGLE_DEG * M_PI / 180.0;
  const double floor_intersection_m =
      ULTRASONIC_HEIGHT_M / tan(down_angle_rad);
  const double projected_m = slant_distance_m * cos(down_angle_rad);

  return projected_m < floor_intersection_m ? projected_m : floor_intersection_m;
}

static double color_approach_distance_m(double slant_distance_m) {
  double approach_m =
      ultrasonic_ground_projection_m(slant_distance_m) -
      COLOR_SENSOR_FORWARD_OFFSET_M;

  if (approach_m < COLOR_APPROACH_MIN_DISTANCE_M) {
    return COLOR_APPROACH_MIN_DISTANCE_M;
  }
  if (approach_m > COLOR_APPROACH_MAX_DISTANCE_M) {
    return COLOR_APPROACH_MAX_DISTANCE_M;
  }
  return approach_m;
}

static int scaled_steps_from_distance_m(double distance_m, double scale) {
  const double raw_steps = (distance_m / NOMINAL_STEP_DISTANCE_M) * scale;

  if (raw_steps > (double)MAX_STEPPER_STEPS) {
    return MAX_STEPPER_STEPS;
  }
  if (raw_steps < (double)MIN_STEPPER_STEPS) {
    return MIN_STEPPER_STEPS;
  }
  return (int)lround(raw_steps);
}

static uint64_t estimate_motion_duration_ms(int16_t left_steps,
                                            int16_t right_steps,
                                            uint16_t left_speed_ticks,
                                            uint16_t right_speed_ticks) {
  const double left_ms =
      (fabs((double)left_steps) * (double)left_speed_ticks * 1000.0) /
      TIMER_TICKS_PER_SECOND;
  const double right_ms =
      (fabs((double)right_steps) * (double)right_speed_ticks * 1000.0) /
      TIMER_TICKS_PER_SECOND;
  const double duration_ms = left_ms > right_ms ? left_ms : right_ms;

  if (duration_ms < 1.0) {
    return 1U;
  }
  return (uint64_t)llround(duration_ms);
}

static void uart_rx_reset(uart_rx_t *rx) {
  rx->length_index = 0U;
  rx->payload_length = 0U;
  rx->payload_index = 0U;
  rx->discarding_payload = false;
}

static uint32_t read_u32_le(const uint8_t bytes[sizeof(uint32_t)]) {
  return ((uint32_t)bytes[0]) | ((uint32_t)bytes[1] << 8U) |
         ((uint32_t)bytes[2] << 16U) | ((uint32_t)bytes[3] << 24U);
}

static void write_u32_le(uint32_t value) {
  uart_send(UART0, (uint8_t)(value & 0xFFU));
  uart_send(UART0, (uint8_t)((value >> 8U) & 0xFFU));
  uart_send(UART0, (uint8_t)((value >> 16U) & 0xFFU));
  uart_send(UART0, (uint8_t)((value >> 24U) & 0xFFU));
}

static void bridge_send_json_frame(const char *json) {
  const uint32_t length = (uint32_t)strlen(json);

  write_u32_le(length);
  for (uint32_t i = 0U; i < length; ++i) {
    uart_send(UART0, (uint8_t)json[i]);
  }
}

static const char *mode_name(control_mode_t mode) {
  switch (mode) {
  case MODE_MOUNTAIN_DETECT:
    return "mountain_detect";
  case MODE_BORDER_SEARCH:
    return "border_search";
  case MODE_ALIGN_LINE:
    return "align_line";
  case MODE_TURN_AWAY_FROM_LINE:
    return "turn_away_from_line";
  case MODE_LEAVE_LINE:
    return "leave_line";
  case MODE_TURN_TO_LINE:
    return "turn_to_line";
  case MODE_BORDER_MAP:
    return "border_map";
  case MODE_COLOR_APPROACH:
    return "color_approach";
  case MODE_COLOR_SAMPLE:
    return "color_sample";
  case MODE_AREA_SWEEP:
    return "area_sweep";
  case MODE_AREA_AVOID:
    return "area_avoid";
  case MODE_IDLE:
  default:
    return "idle";
  }
}

static const char *json_value_start(const char *json, const char *key) {
  const char *p = strstr(json, key);
  if (p == NULL) {
    return NULL;
  }

  p += strlen(key);
  while (*p == ' ') {
    p++;
  }
  if (*p != ':') {
    return NULL;
  }
  p++;
  while (*p == ' ') {
    p++;
  }
  if (*p == '"') {
    p++;
  }
  return p;
}

static bool json_string_value_is(const char *json, const char *key,
                                 const char *value) {
  const char *p = json_value_start(json, key);
  const size_t length = strlen(value);
  char end = '\0';

  if (p == NULL || strncmp(p, value, length) != 0) {
    return false;
  }

  end = p[length];
  return end == '"' || end == ',' || end == '}' || end == ' ' || end == '\0';
}

static bool json_double_value(const char *json, const char *key, double *out) {
  const char *p = json_value_start(json, key);
  char *end = NULL;

  if (p == NULL) {
    return false;
  }

  *out = strtod(p, &end);
  return end != p;
}

static bool command_is_stop(const char *payload) {
  return json_string_value_is(payload, "\"cmd\"", "stop") ||
         json_string_value_is(payload, "\"mode\"", "idle") ||
         json_string_value_is(payload, "\"mode\"", "stop");
}

static bool command_is_mode(const char *payload, const char *mode) {
  return json_string_value_is(payload, "\"cmd\"", "set_mode") &&
         json_string_value_is(payload, "\"mode\"", mode);
}

static bool command_is_mountain_start(const char *payload) {
  return command_is_mode(payload, "mountain_detect") ||
         json_string_value_is(payload, "\"cmd\"", "start_mountain") ||
         json_string_value_is(payload, "\"mode\"", "mountain_detect");
}

static bool command_is_border_start(const char *payload) {
  return json_string_value_is(payload, "\"cmd\"", "start") ||
         json_string_value_is(payload, "\"cmd\"", "start_test") ||
         json_string_value_is(payload, "\"cmd\"", "start_border") ||
         json_string_value_is(payload, "\"mode\"", "border_search") ||
         json_string_value_is(payload, "\"mode\"", "border_map");
}

static bool command_is_area_start(const char *payload) {
  return command_is_mode(payload, "area_sweep") ||
         json_string_value_is(payload, "\"cmd\"", "start_area") ||
         json_string_value_is(payload, "\"mode\"", "area_sweep") ||
         json_string_value_is(payload, "\"mode\"", "area");
}

static bool command_is_map_hint(const char *payload) {
  return json_string_value_is(payload, "\"cmd\"", "map_hint") ||
         json_string_value_is(payload, "\"type\"", "map_hint");
}

static bool pin_detects_black(io_t pin) {
  return gpio_get_level(pin) == BLACK_DETECTED_LEVEL;
}

static ir_readings_t read_ir_sensors(void) {
  ir_readings_t readings;
  readings.left_black = pin_detects_black(LEFT_IR_PIN);
  readings.right_black = pin_detects_black(RIGHT_IR_PIN);
  return readings;
}

static bool any_ir_black(const ir_readings_t *ir) {
  return ir->left_black || ir->right_black;
}

static bool both_ir_black(const ir_readings_t *ir) {
  return ir->left_black && ir->right_black;
}

static const char *json_bool(bool value) { return value ? "true" : "false"; }

static int align_turn_sign_for_ir(const ir_readings_t *ir) {
  if (ir->left_black && !ir->right_black) {
    return -1;
  }
  if (ir->right_black && !ir->left_black) {
    return 1;
  }
  return 1;
}

static void sensor_bar_center(const telemetry_state_t *state, double *x,
                              double *y) {
  *x = state->coord_x + (IR_SENSOR_FORWARD_OFFSET_M * cos(state->theta));
  *y = state->coord_y + (IR_SENSOR_FORWARD_OFFSET_M * sin(state->theta));
}

static void initialize_motion(void) {
  stepper_init();
  stepper_reset();
  stepper_enable();
  sleep_msec(MOTOR_ENABLE_SETTLE_MS);
}

static void reset_and_enable_motion(void) {
  stepper_reset();
  stepper_enable();
  sleep_msec(POST_COMMAND_SETTLE_MS);
}

static void clear_motion(motion_state_t *motion) {
  motion->active = false;
  motion->scan_waiting_to_read = false;
  motion->telemetry_now = true;
}

static void reset_and_clear_motion(motion_state_t *motion) {
  reset_and_enable_motion();
  clear_motion(motion);
}

static void set_motion_target(motion_state_t *motion,
                              const telemetry_state_t *state, double target_x,
                              double target_y, double target_theta) {
  motion->start_x = state->coord_x;
  motion->start_y = state->coord_y;
  motion->start_theta = state->theta;
  motion->target_x = target_x;
  motion->target_y = target_y;
  motion->target_theta = target_theta;
}

static void start_stepper_motion(motion_state_t *motion, int16_t left_steps,
                                 int16_t right_steps,
                                 uint16_t left_speed_ticks,
                                 uint16_t right_speed_ticks) {
  stepper_set_speed(left_speed_ticks, right_speed_ticks);
  sleep_msec(PRE_COMMAND_SETTLE_MS);
  stepper_steps(left_steps, right_steps);

  motion->started_at_ms = monotonic_ms();
  motion->estimated_duration_ms =
      estimate_motion_duration_ms(left_steps, right_steps, left_speed_ticks,
                                  right_speed_ticks);
  motion->active = true;
  motion->telemetry_now = true;
}

#if HAVE_VL53L0X
static uint32_t apply_distance_offset(uint32_t raw_distance_mm) {
  if (raw_distance_mm == UINT32_MAX) {
    return UINT32_MAX;
  }
  if (raw_distance_mm <= TOF_ZERO_OFFSET_MM) {
    return 0U;
  }
  return raw_distance_mm - TOF_ZERO_OFFSET_MM;
}
#endif

static bool initialize_tof(vl53x *sensor) {
#if HAVE_VL53L0X
  switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
  switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
  iic_init(IIC0);
  iic_reset(IIC0);
  sleep_msec(50);

  if (tofPing(IIC0, VL53L0X_ADDR) != 0) {
    printf("ToF ping failed at 0x%02X\n", VL53L0X_ADDR);
    fflush(stdout);
    return false;
  }

  if (tofInit(sensor, IIC0, VL53L0X_ADDR, RANGE_MODE_LONG) != 0) {
    printf("ToF init failed\n");
    fflush(stdout);
    return false;
  }

  printf("ToF ready on SDA/SCL at 0x%02X\n", VL53L0X_ADDR);
  fflush(stdout);
  return true;
#else
  (void)sensor;
  printf("ToF support not compiled: vl53l0x.h was not found.\n");
  fflush(stdout);
  return false;
#endif
}

static bool read_tof_distance_m(vl53x *sensor, double *distance_m) {
#if HAVE_VL53L0X
  const uint32_t raw_distance_mm = tofReadDistance(sensor);
  if (raw_distance_mm == UINT32_MAX) {
    return false;
  }

  const uint32_t corrected_distance_mm = apply_distance_offset(raw_distance_mm);
  *distance_m = (double)corrected_distance_mm / 1000.0;
  return true;
#else
  (void)sensor;
  (void)distance_m;
  return false;
#endif
}

static void set_color_output_scaling_20_percent(void) {
  gpio_set_level(COLOR_S0_PIN, GPIO_LEVEL_HIGH);
  gpio_set_level(COLOR_S1_PIN, GPIO_LEVEL_LOW);
}

static void select_color_filter(gpio_level_t s2, gpio_level_t s3) {
  gpio_set_level(COLOR_S2_PIN, s2);
  gpio_set_level(COLOR_S3_PIN, s3);
}

static uint32_t measure_selected_color_channel(void) {
  uint32_t time_ticks = 0U;

  sleep_msec(FILTER_SETTLE_MS);
  pulsecounter_reset_count(PULSECOUNTER0);
  sleep_msec(SAMPLE_WINDOW_MS);

  return pulsecounter_get_count(PULSECOUNTER0, &time_ticks);
}

static color_sample_t read_color_sample(void) {
  color_sample_t sample;

  set_color_output_scaling_20_percent();

  select_color_filter(GPIO_LEVEL_LOW, GPIO_LEVEL_LOW);
  sample.red = measure_selected_color_channel();

  select_color_filter(GPIO_LEVEL_HIGH, GPIO_LEVEL_HIGH);
  sample.green = measure_selected_color_channel();

  select_color_filter(GPIO_LEVEL_LOW, GPIO_LEVEL_HIGH);
  sample.blue = measure_selected_color_channel();

  return sample;
}

static bool read_ultrasonic_distance_m(double *distance_m) {
  uint64_t deadline = 0U;
  uint64_t echo_start = 0U;
  uint64_t echo_end = 0U;

  gpio_set_level(ULTRASONIC_TRIG_PIN, GPIO_LEVEL_LOW);
  sleep_usec_exact(2);
  gpio_set_level(ULTRASONIC_TRIG_PIN, GPIO_LEVEL_HIGH);
  sleep_usec_exact(TRIGGER_PULSE_US);
  gpio_set_level(ULTRASONIC_TRIG_PIN, GPIO_LEVEL_LOW);

  deadline = monotonic_usec() + ECHO_START_TIMEOUT_US;
  while (gpio_get_level(ULTRASONIC_ECHO_PIN) == GPIO_LEVEL_LOW) {
    if (monotonic_usec() >= deadline) {
      return false;
    }
  }
  echo_start = monotonic_usec();

  deadline = echo_start + ECHO_END_TIMEOUT_US;
  while (gpio_get_level(ULTRASONIC_ECHO_PIN) == GPIO_LEVEL_HIGH) {
    if (monotonic_usec() >= deadline) {
      return false;
    }
  }
  echo_end = monotonic_usec();

  if (echo_end <= echo_start) {
    return false;
  }

  *distance_m =
      ((double)(echo_end - echo_start) * SOUND_SPEED_M_PER_S) / 2000000.0;
  return true;
}

static color_label_t classify_color_sample(const color_sample_t *sample,
                                           double distance_cm) {
  const double features[COLOR_MODEL_FEATURES] = {
      (double)sample->red, (double)sample->green, (double)sample->blue,
      distance_cm > 0.0 ? distance_cm : COLOR_MODEL_DIST_FILL_CM};
  color_label_t best_label = DETECT_RED;
  double best_score = -1.0e100;

  for (uint16_t label = 0U; label < COLOR_MODEL_LABELS; ++label) {
    double score = COLOR_MODEL_B[label];
    for (uint16_t feature = 0U; feature < COLOR_MODEL_FEATURES; ++feature) {
      const double std = COLOR_MODEL_STD[feature] != 0.0
                             ? COLOR_MODEL_STD[feature]
                             : 1.0;
      const double z = (features[feature] - COLOR_MODEL_MEAN[feature]) / std;
      score += COLOR_MODEL_W[label][feature] * z;
    }

    if (score > best_score) {
      best_score = score;
      best_label = (color_label_t)label;
    }
  }

  return best_label;
}

static const char *color_name(color_label_t label) {
  switch (label) {
  case DETECT_RED:
    return "red";
  case DETECT_GREEN:
    return "green";
  case DETECT_BLUE:
    return "blue";
  case DETECT_WHITE:
    return "white";
  case DETECT_BLACK:
  default:
    return "black";
  }
}

static color_label_t read_averaged_color(color_sample_t *average,
                                         uint16_t *sample_count,
                                         double distance_cm) {
  uint64_t red_sum = 0U;
  uint64_t green_sum = 0U;
  uint64_t blue_sum = 0U;
  const uint64_t deadline_ms = monotonic_ms() + COLOR_SAMPLE_DURATION_MS;

  *sample_count = 0U;
  average->red = 0U;
  average->green = 0U;
  average->blue = 0U;

  while (keep_running && monotonic_ms() < deadline_ms) {
    const color_sample_t sample = read_color_sample();

    if (sample.red != 0U || sample.green != 0U || sample.blue != 0U) {
      red_sum += sample.red;
      green_sum += sample.green;
      blue_sum += sample.blue;
      ++(*sample_count);
    }
  }

  if (*sample_count == 0U) {
    return DETECT_BLACK;
  }

  average->red = (uint32_t)(red_sum / *sample_count);
  average->green = (uint32_t)(green_sum / *sample_count);
  average->blue = (uint32_t)(blue_sum / *sample_count);
  return classify_color_sample(average, distance_cm);
}

static bool initialize_color_ultrasonic(void) {
  gpio_set_direction(COLOR_S0_PIN, GPIO_DIR_OUTPUT);
  gpio_set_direction(COLOR_S1_PIN, GPIO_DIR_OUTPUT);
  gpio_set_direction(COLOR_S2_PIN, GPIO_DIR_OUTPUT);
  gpio_set_direction(COLOR_S3_PIN, GPIO_DIR_OUTPUT);
  gpio_set_direction(COLOR_OUT_PIN, GPIO_DIR_INPUT);

  gpio_set_direction(ULTRASONIC_TRIG_PIN, GPIO_DIR_OUTPUT);
  gpio_set_direction(ULTRASONIC_ECHO_PIN, GPIO_DIR_INPUT);
  gpio_set_level(ULTRASONIC_TRIG_PIN, GPIO_LEVEL_LOW);

  switchbox_set_pin(COLOR_OUT_PIN, SWB_TIMER_IC0);
  pulsecounter_init(PULSECOUNTER0);
  pulsecounter_set_edge(PULSECOUNTER0, GPIO_LEVEL_HIGH);
  pulsecounter_reset_count(PULSECOUNTER0);
  set_color_output_scaling_20_percent();
  return true;
}

static void send_status(const char *status, const telemetry_state_t *state,
                        bool moving, control_mode_t mode,
                        const ir_readings_t *ir) {
  char json[MQTT_JSON_MAX];
  snprintf(json, sizeof(json),
           "{\"type\":\"status\",\"robot\":\"" MODULE_NUMBER
           "\",\"t_ms\":%llu,\"status\":\"%s\",\"coordX\":%.6f,"
           "\"coordY\":%.6f,\"theta\":%.6f,\"moving\":%s,\"done\":%s,"
           "\"mode\":\"%s\",\"irLeft\":%s,\"irRight\":%s}",
           (unsigned long long)monotonic_ms(), status, state->coord_x,
           state->coord_y, state->theta, json_bool(moving),
           json_bool(mode == MODE_IDLE), mode_name(mode),
           json_bool(ir->left_black), json_bool(ir->right_black));
  bridge_send_json_frame(json);
  printf("<< Sent framed status: %s\n", json);
  fflush(stdout);
}

static void send_border_point(const char *event, const telemetry_state_t *state,
                              const ir_readings_t *ir, double x, double y,
                              double line_theta) {
  char json[MQTT_JSON_MAX];
  snprintf(json, sizeof(json),
           "{\"type\":\"border_point\",\"robot\":\"" MODULE_NUMBER
           "\",\"t_ms\":%llu,\"event\":\"%s\",\"coordX\":%.6f,"
           "\"coordY\":%.6f,\"theta\":%.6f,\"borderX\":%.6f,"
           "\"borderY\":%.6f,\"lineTheta\":%.6f,\"irLeft\":%s,"
           "\"irRight\":%s}",
           (unsigned long long)monotonic_ms(), event, state->coord_x,
           state->coord_y, state->theta, x, y, line_theta,
           json_bool(ir->left_black), json_bool(ir->right_black));
  bridge_send_json_frame(json);
  printf("<< Sent border point: %s\n", json);
  fflush(stdout);
}

static void send_sensor_border_point(const char *event,
                                     const telemetry_state_t *state,
                                     const ir_readings_t *ir,
                                     double line_theta, double *x,
                                     double *y) {
  sensor_bar_center(state, x, y);
  send_border_point(event, state, ir, *x, *y, line_theta);
}

static void send_border_line(const telemetry_state_t *state,
                             const ir_readings_t *ir, double x, double y,
                             double line_theta) {
  char json[MQTT_JSON_MAX];
  snprintf(json, sizeof(json),
           "{\"type\":\"border_line\",\"robot\":\"" MODULE_NUMBER
           "\",\"t_ms\":%llu,\"coordX\":%.6f,\"coordY\":%.6f,"
           "\"theta\":%.6f,\"borderX\":%.6f,\"borderY\":%.6f,"
           "\"lineTheta\":%.6f,\"irLeft\":%s,\"irRight\":%s}",
           (unsigned long long)monotonic_ms(), state->coord_x, state->coord_y,
           state->theta, x, y, line_theta, json_bool(ir->left_black),
           json_bool(ir->right_black));
  bridge_send_json_frame(json);
  printf("<< Sent border line: %s\n", json);
  fflush(stdout);
}

static void send_mountain_detection(const telemetry_state_t *state,
                                    double distance_m) {
  char json[MQTT_JSON_MAX];
  const double det_x = state->coord_x + (distance_m * cos(state->theta));
  const double det_y = state->coord_y + (distance_m * sin(state->theta));

  snprintf(json, sizeof(json),
           "{\"type\":\"mountain_detection\",\"robot\":\"" MODULE_NUMBER
           "\",\"t_ms\":%llu,\"coordX\":%.6f,\"coordY\":%.6f,"
           "\"theta\":%.6f,\"distance_m\":%.6f,\"detX\":%.6f,"
           "\"detY\":%.6f}",
           (unsigned long long)monotonic_ms(), state->coord_x, state->coord_y,
           state->theta, distance_m, det_x, det_y);
  bridge_send_json_frame(json);
  printf("<< Sent mountain detection: %s\n", json);
  fflush(stdout);
}

static void send_color_detection(const telemetry_state_t *state,
                                 const color_object_state_t *color_object,
                                 color_label_t label,
                                 const color_sample_t *average,
                                 uint16_t sample_count) {
  char json[MQTT_JSON_MAX];

  snprintf(json, sizeof(json),
           "{\"type\":\"color_detection\",\"robot\":\"" MODULE_NUMBER
           "\",\"t_ms\":%llu,\"coordX\":%.6f,\"coordY\":%.6f,"
           "\"theta\":%.6f,\"distance_m\":%.6f,\"colorX\":%.6f,"
           "\"colorY\":%.6f,\"color\":\"%s\",\"red\":%u,\"green\":%u,"
           "\"blue\":%u,\"samples\":%u}",
           (unsigned long long)monotonic_ms(), state->coord_x, state->coord_y,
           state->theta, color_object->trigger_distance_m,
           color_object->target_x, color_object->target_y, color_name(label),
           average->red, average->green, average->blue, sample_count);
  bridge_send_json_frame(json);
  printf("<< Sent color detection: %s\n", json);
  fflush(stdout);
}

static void send_crater_detection(const telemetry_state_t *state,
                                  const ir_readings_t *ir) {
  char json[MQTT_JSON_MAX];
  double crater_x = 0.0;
  double crater_y = 0.0;

  sensor_bar_center(state, &crater_x, &crater_y);
  snprintf(json, sizeof(json),
           "{\"type\":\"crater_detection\",\"robot\":\"" MODULE_NUMBER
           "\",\"t_ms\":%llu,\"coordX\":%.6f,\"coordY\":%.6f,"
           "\"theta\":%.6f,\"craterX\":%.6f,\"craterY\":%.6f,"
           "\"irLeft\":%s,\"irRight\":%s}",
           (unsigned long long)monotonic_ms(), state->coord_x, state->coord_y,
           state->theta, crater_x, crater_y, json_bool(ir->left_black),
           json_bool(ir->right_black));
  bridge_send_json_frame(json);
  printf("<< Sent crater detection: %s\n", json);
  fflush(stdout);
}

static void remember_border_sample(border_state_t *border, double x, double y) {
  if (border->sample_count < AREA_MAX_BORDER_SAMPLES) {
    border->sample_x[border->sample_count] = x;
    border->sample_y[border->sample_count] = y;
    border->sample_count++;
    return;
  }

  const uint16_t index = border->sample_count % AREA_MAX_BORDER_SAMPLES;
  border->sample_x[index] = x;
  border->sample_y[index] = y;
  border->sample_count++;
}

static double nearest_remembered_border_m(const border_state_t *border,
                                          double x, double y) {
  const uint16_t count = border->sample_count < AREA_MAX_BORDER_SAMPLES
                             ? border->sample_count
                             : AREA_MAX_BORDER_SAMPLES;
  double best = 1000.0;

  for (uint16_t i = 0U; i < count; ++i) {
    const double distance_m = hypot(border->sample_x[i] - x,
                                    border->sample_y[i] - y);
    if (distance_m < best) {
      best = distance_m;
    }
  }

  return best;
}

static bool map_hint_is_fresh(const map_hint_state_t *hint) {
  return hint->valid &&
         monotonic_ms() - hint->received_at_ms <= MAP_HINT_MAX_AGE_MS;
}

static bool area_ir_is_near_border(const border_state_t *border,
                                   const map_hint_state_t *hint,
                                   const telemetry_state_t *state) {
  double sensor_x = 0.0;
  double sensor_y = 0.0;

  sensor_bar_center(state, &sensor_x, &sensor_y);
  if (nearest_remembered_border_m(border, sensor_x, sensor_y) <=
      AREA_BORDER_CLOSE_M) {
    return true;
  }

  return map_hint_is_fresh(hint) &&
         hint->nearest_border_distance_m <= AREA_BORDER_CLOSE_M;
}

static bool area_has_object_ahead(const map_hint_state_t *hint) {
  return map_hint_is_fresh(hint) &&
         hint->obstacle_distance_m > 0.0 &&
         hint->obstacle_distance_m <= AREA_OBJECT_AVOID_DISTANCE_M &&
         fabs(hint->obstacle_bearing_rad) <= AREA_OBJECT_AVOID_BEARING_RAD;
}

static void scan_current_heading(vl53x *sensor, const telemetry_state_t *state) {
  double distance_m = 0.0;
  if (!read_tof_distance_m(sensor, &distance_m)) {
    printf("ToF read failed at theta %.3f\n", state->theta);
    fflush(stdout);
    return;
  }

  printf("ToF theta %.3f distance %.3f m\n", state->theta, distance_m);
  fflush(stdout);
  if (distance_m < TOF_MAX_MOUNTAIN_DISTANCE_M) {
    send_mountain_detection(state, distance_m);
  }
}

static const char *periodic_status(control_mode_t mode,
                                   const motion_state_t *motion,
                                   bool tof_ready) {
  if (mode == MODE_MOUNTAIN_DETECT) {
    return "mountain_detect";
  }
  if (mode == MODE_COLOR_APPROACH) {
    return "color_approach";
  }
  if (mode == MODE_COLOR_SAMPLE) {
    return "color_sample";
  }
  if (mode == MODE_AREA_SWEEP) {
    return "area_sweep";
  }
  if (mode == MODE_AREA_AVOID) {
    return "area_avoid";
  }
  if (!tof_ready) {
    return "ready_no_tof";
  }
  if (mode == MODE_IDLE) {
    return "idle";
  }
  return motion->active ? "moving" : mode_name(mode);
}

static void start_straight(motion_state_t *motion, telemetry_state_t *state,
                           double distance_m, uint16_t speed_ticks) {
  const int steps = scaled_steps_from_distance_m(distance_m, STEP_DISTANCE_SCALE);
  const double target_x = state->coord_x + (distance_m * cos(state->theta));
  const double target_y = state->coord_y + (distance_m * sin(state->theta));

  set_motion_target(motion, state, target_x, target_y, state->theta);
  start_stepper_motion(motion, (int16_t)steps, (int16_t)steps, speed_ticks,
                       speed_ticks);
}

static void start_turn(motion_state_t *motion, telemetry_state_t *state,
                       double angle_rad, uint16_t speed_ticks) {
  const double wheel_travel_m = 0.5 * WHEEL_BASE_M * fabs(angle_rad);
  const int steps = scaled_steps_from_distance_m(wheel_travel_m, TURN_STEP_SCALE);
  const int left_steps = angle_rad > 0.0 ? -steps : steps;
  const int right_steps = angle_rad > 0.0 ? steps : -steps;

  set_motion_target(motion, state, state->coord_x, state->coord_y,
                    state->theta + angle_rad);
  start_stepper_motion(motion, (int16_t)left_steps, (int16_t)right_steps,
                       speed_ticks, speed_ticks);
}

static double area_x_forward_remaining_m(const area_state_t *area,
                                         const telemetry_state_t *state) {
  const double limit = area->sweep_sign >= 0 ? area->max_x : area->min_x;
  const double remaining = (limit - state->coord_x) * (double)area->sweep_sign;
  return remaining > 0.0 ? remaining : 0.0;
}

static double area_y_forward_remaining_m(const area_state_t *area,
                                         const telemetry_state_t *state) {
  const double limit = area->row_sign >= 0 ? area->max_y : area->min_y;
  const double remaining = (limit - state->coord_y) * (double)area->row_sign;
  return remaining > 0.0 ? remaining : 0.0;
}

static double area_current_sweep_theta(const area_state_t *area) {
  return normalize_angle(area->sweep_theta +
                         (area->sweep_sign >= 0 ? 0.0 : M_PI));
}

static double area_current_row_theta(const area_state_t *area) {
  return normalize_angle(area->sweep_theta +
                         (area->row_sign >= 0 ? M_PI_2 : -M_PI_2));
}

static void start_area_sweep(motion_state_t *motion, area_state_t *area,
                             telemetry_state_t *state, control_mode_t *mode,
                             const ir_readings_t *ir) {
  reset_and_clear_motion(motion);

  if (area->max_x <= area->min_x) {
    area->min_x = AREA_DEFAULT_MIN_X_M;
    area->max_x = AREA_DEFAULT_MAX_X_M;
  }
  if (area->max_y <= area->min_y) {
    area->min_y = AREA_DEFAULT_MIN_Y_M;
    area->max_y = AREA_DEFAULT_MAX_Y_M;
  }
  if (area->row_spacing_m <= 0.0) {
    area->row_spacing_m = AREA_ROW_SPACING_M;
  }
  if (area->sweep_step_m <= 0.0) {
    area->sweep_step_m = AREA_SWEEP_STEP_M;
  }

  area->min_x += AREA_MARGIN_M;
  area->max_x -= AREA_MARGIN_M;
  area->min_y += AREA_MARGIN_M;
  area->max_y -= AREA_MARGIN_M;

  if (area->max_x <= area->min_x || area->max_y <= area->min_y) {
    *mode = MODE_IDLE;
    send_status("area_bounds_too_small", state, false, *mode, ir);
    return;
  }

  area->active = true;
  area->phase = AREA_PHASE_ALIGN;
  area->sweep_sign =
      fabs(state->coord_x - area->min_x) <= fabs(state->coord_x - area->max_x)
          ? 1
          : -1;
  area->row_sign =
      fabs(state->coord_y - area->min_y) <= fabs(state->coord_y - area->max_y)
          ? 1
          : -1;
  *mode = MODE_AREA_SWEEP;
  send_status("area_sweep_started", state, false, *mode, ir);
}

static void area_start_crater_avoidance(motion_state_t *motion,
                                        area_state_t *area,
                                        telemetry_state_t *state,
                                        control_mode_t *mode,
                                        const ir_readings_t *ir) {
  reset_and_clear_motion(motion);
  send_crater_detection(state, ir);
  area->phase = AREA_PHASE_AVOID_BACKUP;
  *mode = MODE_AREA_AVOID;
  send_status("crater_detected_avoiding", state, true, *mode, ir);
}

static void area_start_object_avoidance(motion_state_t *motion,
                                        area_state_t *area,
                                        telemetry_state_t *state,
                                        control_mode_t *mode,
                                        const ir_readings_t *ir) {
  reset_and_clear_motion(motion);
  area->phase = AREA_PHASE_AVOID_BACKUP;
  *mode = MODE_AREA_AVOID;
  send_status("mapped_object_ahead_avoiding", state, true, *mode, ir);
}

static void poll_area_autonomy(motion_state_t *motion, border_state_t *border,
                               area_state_t *area,
                               const map_hint_state_t *map_hint,
                               telemetry_state_t *state,
                               control_mode_t *mode) {
  const ir_readings_t ir = read_ir_sensors();

  if (!area->active ||
      (*mode != MODE_AREA_SWEEP && *mode != MODE_AREA_AVOID)) {
    return;
  }

  if (*mode == MODE_AREA_SWEEP && any_ir_black(&ir)) {
    if (!area_ir_is_near_border(border, map_hint, state)) {
      area_start_crater_avoidance(motion, area, state, mode, &ir);
      return;
    }

    if (*mode == MODE_AREA_SWEEP) {
      reset_and_clear_motion(motion);
      area->phase = AREA_PHASE_TURN_TO_ROW;
      send_status("area_border_close_turning_row", state, true, *mode, &ir);
      return;
    }
  }

  if (*mode == MODE_AREA_SWEEP && motion->active &&
      area_has_object_ahead(map_hint)) {
    area_start_object_avoidance(motion, area, state, mode, &ir);
    return;
  }

  if (motion->active) {
    return;
  }

  switch (area->phase) {
  case AREA_PHASE_ALIGN:
    *mode = MODE_AREA_SWEEP;
    area->phase = AREA_PHASE_SWEEP;
    send_status("area_aligning", state, true, *mode, &ir);
    start_turn(motion, state,
               shortest_delta(state->theta, area_current_sweep_theta(area)),
               TURN_PULSE_DELAY_TICKS);
    return;

  case AREA_PHASE_SWEEP: {
    const double remaining_m = area_x_forward_remaining_m(area, state);
    if (remaining_m <= AREA_BOUNDARY_EPS_M) {
      area->phase = AREA_PHASE_TURN_TO_ROW;
      return;
    }

    const double step_m =
        remaining_m < area->sweep_step_m ? remaining_m : area->sweep_step_m;
    start_straight(motion, state, step_m, MAP_PULSE_DELAY_TICKS);
    return;
  }

  case AREA_PHASE_TURN_TO_ROW:
    if (area_y_forward_remaining_m(area, state) <= AREA_BOUNDARY_EPS_M) {
      area->phase = AREA_PHASE_DONE;
      return;
    }
    area->phase = AREA_PHASE_ROW_SHIFT;
    send_status("area_turning_to_next_row", state, true, *mode, &ir);
    start_turn(motion, state,
               shortest_delta(state->theta, area_current_row_theta(area)),
               TURN_PULSE_DELAY_TICKS);
    return;

  case AREA_PHASE_ROW_SHIFT: {
    const double remaining_m = area_y_forward_remaining_m(area, state);
    const double shift_m =
        remaining_m < area->row_spacing_m ? remaining_m : area->row_spacing_m;

    area->phase = AREA_PHASE_TURN_TO_SWEEP;
    send_status("area_shifting_row", state, true, *mode, &ir);
    start_straight(motion, state, shift_m, MAP_PULSE_DELAY_TICKS);
    return;
  }

  case AREA_PHASE_TURN_TO_SWEEP:
    area->sweep_sign *= -1;
    area->phase = AREA_PHASE_SWEEP;
    send_status("area_turning_to_sweep", state, true, *mode, &ir);
    start_turn(motion, state,
               shortest_delta(state->theta, area_current_sweep_theta(area)),
               TURN_PULSE_DELAY_TICKS);
    return;

  case AREA_PHASE_AVOID_BACKUP:
    area->phase = AREA_PHASE_AVOID_TURN;
    start_straight(motion, state, AREA_AVOID_BACKUP_M,
                   CLEARANCE_PULSE_DELAY_TICKS);
    return;

  case AREA_PHASE_AVOID_TURN:
    area->phase = AREA_PHASE_AVOID_FORWARD;
    start_turn(motion, state, (double)area->row_sign * AREA_AVOID_TURN_RAD,
               TURN_PULSE_DELAY_TICKS);
    return;

  case AREA_PHASE_AVOID_FORWARD:
    area->phase = AREA_PHASE_ALIGN;
    send_status("area_avoidance_complete", state, true, *mode, &ir);
    start_straight(motion, state, AREA_AVOID_FORWARD_M,
                   CLEARANCE_PULSE_DELAY_TICKS);
    return;

  case AREA_PHASE_DONE:
  default:
    area->active = false;
    *mode = MODE_IDLE;
    send_status("area_sweep_complete", state, false, *mode, &ir);
    return;
  }
}

static void start_scan_step(motion_state_t *motion, telemetry_state_t *state) {
  const double wheel_travel_m = 0.5 * WHEEL_BASE_M * SCAN_STEP_RAD;
  const int steps = scaled_steps_from_distance_m(wheel_travel_m, TURN_STEP_SCALE);

  set_motion_target(motion, state, state->coord_x, state->coord_y,
                    state->theta + SCAN_STEP_RAD);
  start_stepper_motion(motion, (int16_t)-steps, (int16_t)steps,
                       SCAN_PULSE_DELAY_TICKS, SCAN_PULSE_DELAY_TICKS);
  motion->scan_waiting_to_read = false;
}

static void start_mountain_scan(motion_state_t *motion, telemetry_state_t *state,
                                vl53x *sensor) {
  clear_motion(motion);
  motion->scan_step_index = 0U;
  motion->scan_step_count = (uint16_t)lround((2.0 * M_PI) / SCAN_STEP_RAD);

  scan_current_heading(sensor, state);
  start_scan_step(motion, state);
}

static void poll_motion(motion_state_t *motion, telemetry_state_t *state,
                        control_mode_t *mode, vl53x *sensor) {
  const uint64_t now_ms = monotonic_ms();

  if (*mode == MODE_MOUNTAIN_DETECT && motion->scan_waiting_to_read &&
      now_ms >= motion->scan_read_at_ms) {
    const ir_readings_t ir = read_ir_sensors();
    scan_current_heading(sensor, state);
    motion->scan_waiting_to_read = false;

    motion->scan_step_index++;
    if (motion->scan_step_index >= motion->scan_step_count) {
      reset_and_enable_motion();
      motion->active = false;
      motion->scan_waiting_to_read = false;
      *mode = MODE_IDLE;
      send_status("mountain_scan_complete", state, false, *mode, &ir);
      motion->telemetry_now = false;
      return;
    }

    start_scan_step(motion, state);
    return;
  }

  if (!motion->active) {
    return;
  }

  if (stepper_steps_done()) {
    state->coord_x = motion->target_x;
    state->coord_y = motion->target_y;
    state->theta = motion->target_theta;
    motion->active = false;
    motion->telemetry_now = true;
    sleep_msec(POST_COMMAND_SETTLE_MS);

    if (*mode == MODE_MOUNTAIN_DETECT) {
      motion->scan_waiting_to_read = true;
      motion->scan_read_at_ms = monotonic_ms() + SCAN_SETTLE_MS;
    }
    return;
  }

  const uint64_t elapsed_ms =
      (now_ms >= motion->started_at_ms) ? (now_ms - motion->started_at_ms) : 0U;
  const double progress =
      motion->estimated_duration_ms == 0U
          ? 1.0
          : (double)elapsed_ms / (double)motion->estimated_duration_ms;
  const double p = clamp01(progress);

  state->coord_x = motion->start_x + ((motion->target_x - motion->start_x) * p);
  state->coord_y = motion->start_y + ((motion->target_y - motion->start_y) * p);
  state->theta =
      motion->start_theta + ((motion->target_theta - motion->start_theta) * p);
}

static void stop_motion(motion_state_t *motion, telemetry_state_t *state,
                        control_mode_t *mode, const ir_readings_t *ir) {
  reset_and_clear_motion(motion);
  *mode = MODE_IDLE;
  send_status("stopped", state, false, *mode, ir);
}

static bool mode_allows_color_detection(control_mode_t mode) {
  return mode == MODE_BORDER_SEARCH || mode == MODE_BORDER_MAP ||
         mode == MODE_AREA_SWEEP;
}

static void poll_color_object(motion_state_t *motion,
                              color_object_state_t *color_object,
                              area_state_t *area, telemetry_state_t *state,
                              control_mode_t *mode) {
  const uint64_t now_ms = monotonic_ms();

  if (*mode == MODE_COLOR_APPROACH) {
    if (motion->active) {
      return;
    }

    const ir_readings_t ir = read_ir_sensors();
    color_sample_t average = {0U, 0U, 0U};
    uint16_t sample_count = 0U;
    color_label_t label;

    *mode = MODE_COLOR_SAMPLE;
    send_status("color_sampling", state, false, *mode, &ir);
    label = read_averaged_color(&average, &sample_count,
                                color_object->trigger_distance_m * 100.0);
    send_color_detection(state, color_object, label, &average, sample_count);

    clear_motion(motion);
    if (color_object->resume_mode == MODE_AREA_SWEEP ||
        color_object->resume_mode == MODE_AREA_AVOID || area->active) {
      area->active = true;
      area->phase = AREA_PHASE_AVOID_BACKUP;
      *mode = MODE_AREA_AVOID;
      send_status("color_sample_complete_avoiding_block", state, true, *mode,
                  &ir);
    } else {
      *mode = MODE_IDLE;
      send_status("color_sample_complete", state, false, *mode, &ir);
    }
    return;
  }

  if (!mode_allows_color_detection(*mode) || !motion->active ||
      now_ms < color_object->next_ultrasonic_poll_ms) {
    return;
  }
  color_object->next_ultrasonic_poll_ms = now_ms + COLOR_ULTRASONIC_POLL_MS;

  double distance_m = 0.0;
  if (!read_ultrasonic_distance_m(&distance_m) ||
      distance_m > COLOR_TRIGGER_DISTANCE_M) {
    return;
  }

  const ir_readings_t ir = read_ir_sensors();
  const double ground_distance_m = ultrasonic_ground_projection_m(distance_m);
  const double approach_distance_m = color_approach_distance_m(distance_m);
  reset_and_clear_motion(motion);

  color_object->trigger_distance_m = distance_m;
  color_object->target_x =
      state->coord_x + (ground_distance_m * cos(state->theta));
  color_object->target_y =
      state->coord_y + (ground_distance_m * sin(state->theta));
  color_object->resume_mode = *mode;

  *mode = MODE_COLOR_APPROACH;
  send_status("color_object_detected", state, true, *mode, &ir);
  start_straight(motion, state, approach_distance_m,
                 COLOR_APPROACH_PULSE_DELAY_TICKS);
}

static void start_border_search(motion_state_t *motion, border_state_t *border,
                                telemetry_state_t *state, control_mode_t *mode,
                                const ir_readings_t *ir) {
  reset_and_clear_motion(motion);
  border->align_turn_sign = ANTICLOCKWISE_FOLLOW_TURN_SIGN;
  border->follow_turn_sign = ANTICLOCKWISE_FOLLOW_TURN_SIGN;
  border->first_left_black = false;
  border->first_right_black = false;
  border->loop_start_set = false;
  border->align_steps = 0U;
  border->map_steps = 0U;
  border->corner_count = 0U;
  border->align_start_theta = state->theta;
  border->loop_start_x = state->coord_x;
  border->loop_start_y = state->coord_y;
  border->line_anchor_x = state->coord_x;
  border->line_anchor_y = state->coord_y;
  border->line_normal_theta = state->theta;
  border->line_theta = state->theta;
  border->follow_theta = state->theta;
  border->line_distance_m = 0.0;
  border->total_mapped_distance_m = 0.0;
  border->sample_count = 0U;
  *mode = MODE_BORDER_SEARCH;
  send_status("border_search_started", state, false, *mode, ir);
}

static void begin_alignment(control_mode_t *mode, border_state_t *border,
                            const telemetry_state_t *state,
                            const ir_readings_t *ir) {
  border->align_turn_sign = align_turn_sign_for_ir(ir);
  border->follow_turn_sign = ANTICLOCKWISE_FOLLOW_TURN_SIGN;
  border->first_left_black = ir->left_black;
  border->first_right_black = ir->right_black;
  border->align_steps = 0U;
  border->align_start_theta = state->theta;
  sensor_bar_center(state, &border->line_anchor_x, &border->line_anchor_y);
  border->line_normal_theta = state->theta;
  border->line_theta = state->theta + ((double)border->follow_turn_sign *
                                       TURN_TO_LINE_RAD);
  border->follow_theta =
      normalize_angle(border->line_theta +
                      ((double)border->follow_turn_sign * WALL_OVERTURN_RAD));
  border->line_distance_m = 0.0;
  *mode = MODE_ALIGN_LINE;
}

static void remember_line(border_state_t *border,
                          const telemetry_state_t *state) {
  double aligned_theta = state->theta;
  if (border->first_left_black != border->first_right_black) {
    aligned_theta = angle_midpoint(border->align_start_theta, state->theta);
  }

  sensor_bar_center(state, &border->line_anchor_x, &border->line_anchor_y);
  border->line_normal_theta = normalize_angle(aligned_theta);
  border->line_theta =
      normalize_angle(aligned_theta +
                      ((double)border->follow_turn_sign * TURN_TO_LINE_RAD));
  border->follow_theta =
      normalize_angle(border->line_theta +
                      ((double)border->follow_turn_sign * WALL_OVERTURN_RAD));
  border->line_distance_m = 0.0;
}

static void remember_loop_start(border_state_t *border) {
  if (border->loop_start_set) {
    return;
  }

  border->loop_start_x = border->line_anchor_x;
  border->loop_start_y = border->line_anchor_y;
  border->loop_start_set = true;
}

static bool border_loop_closed(const border_state_t *border, double x,
                               double y) {
  if (!border->loop_start_set || border->corner_count < LOOP_MIN_CORNERS ||
      border->total_mapped_distance_m < LOOP_MIN_DISTANCE_M) {
    return false;
  }

  return hypot(x - border->loop_start_x, y - border->loop_start_y) <=
         LOOP_CLOSE_RADIUS_M;
}

static double point_segment_distance_m(double px, double py, double ax,
                                       double ay, double bx, double by) {
  const double vx = bx - ax;
  const double vy = by - ay;
  const double wx = px - ax;
  const double wy = py - ay;
  const double segment_len_sq = (vx * vx) + (vy * vy);
  double t = 0.0;

  if (segment_len_sq <= 1.0e-12) {
    return hypot(px - ax, py - ay);
  }

  t = ((wx * vx) + (wy * vy)) / segment_len_sq;
  if (t < 0.0) {
    t = 0.0;
  } else if (t > 1.0) {
    t = 1.0;
  }

  return hypot(px - (ax + (t * vx)), py - (ay + (t * vy)));
}

static bool border_path_revisited(const border_state_t *border, double x,
                                  double y) {
  const uint16_t count = border->sample_count < AREA_MAX_BORDER_SAMPLES
                             ? border->sample_count
                             : AREA_MAX_BORDER_SAMPLES;
  const uint16_t usable_count =
      count > LOOP_REVISIT_IGNORE_RECENT_SAMPLES
          ? (uint16_t)(count - LOOP_REVISIT_IGNORE_RECENT_SAMPLES)
          : 0U;

  if (!border->loop_start_set ||
      border->corner_count < LOOP_REVISIT_MIN_CORNERS ||
      border->total_mapped_distance_m < LOOP_MIN_DISTANCE_M ||
      usable_count == 0U) {
    return false;
  }

  for (uint16_t i = 0U; i < usable_count; ++i) {
    if (hypot(border->sample_x[i] - x, border->sample_y[i] - y) <=
        LOOP_REVISIT_RADIUS_M) {
      return true;
    }
  }

  for (uint16_t i = 1U; i < usable_count; ++i) {
    if (point_segment_distance_m(x, y, border->sample_x[i - 1U],
                                 border->sample_y[i - 1U],
                                 border->sample_x[i], border->sample_y[i]) <=
        LOOP_REVISIT_RADIUS_M) {
      return true;
    }
  }

  return false;
}

static bool border_finished_by_geometry(const border_state_t *border, double x,
                                        double y) {
  return border_loop_closed(border, x, y) || border_path_revisited(border, x, y);
}

static bool line_alignment_complete(const border_state_t *border,
                                    const ir_readings_t *ir) {
  if (both_ir_black(ir)) {
    return true;
  }
  if (border->first_left_black && !border->first_right_black) {
    return ir->right_black;
  }
  if (border->first_right_black && !border->first_left_black) {
    return ir->left_black;
  }
  return false;
}

static void poll_border_autonomy(motion_state_t *motion, border_state_t *border,
                                 telemetry_state_t *state,
                                 control_mode_t *mode) {
  const ir_readings_t ir = read_ir_sensors();

  if (*mode == MODE_BORDER_SEARCH && motion->active && any_ir_black(&ir)) {
    double x = 0.0;
    double y = 0.0;
    reset_and_clear_motion(motion);
    send_sensor_border_point("tape_hit", state, &ir, state->theta, &x, &y);
    remember_border_sample(border, x, y);
    begin_alignment(mode, border, state, &ir);
    send_status("aligning_to_tape", state, false, *mode, &ir);
    return;
  }

  if (*mode == MODE_BORDER_MAP && motion->active && any_ir_black(&ir) &&
      border->map_steps >= MAP_CORNER_GRACE_STEPS) {
    double x = 0.0;
    double y = 0.0;
    reset_and_clear_motion(motion);
    send_sensor_border_point("corner_hit", state, &ir, state->theta, &x, &y);
    remember_border_sample(border, x, y);
    if (border_finished_by_geometry(border, x, y)) {
      send_status("border_path_revisited", state, false, MODE_IDLE, &ir);
      *mode = MODE_IDLE;
      return;
    }
    border->corner_count++;
    begin_alignment(mode, border, state, &ir);
    send_status("corner_aligning", state, false, *mode, &ir);
    return;
  }

  if (motion->active) {
    return;
  }

  switch (*mode) {
  case MODE_IDLE:
  case MODE_MOUNTAIN_DETECT:
  case MODE_COLOR_APPROACH:
  case MODE_COLOR_SAMPLE:
  case MODE_AREA_SWEEP:
  case MODE_AREA_AVOID:
    return;

  case MODE_BORDER_SEARCH:
    if (any_ir_black(&ir)) {
      double x = 0.0;
      double y = 0.0;
      send_sensor_border_point("tape_hit", state, &ir, state->theta, &x, &y);
      remember_border_sample(border, x, y);
      begin_alignment(mode, border, state, &ir);
      send_status("aligning_to_tape", state, false, *mode, &ir);
      return;
    }
    start_straight(motion, state, SEARCH_STEP_M, SEARCH_PULSE_DELAY_TICKS);
    return;

  case MODE_ALIGN_LINE:
    if (line_alignment_complete(border, &ir)) {
      remember_line(border, state);
      remember_loop_start(border);
      remember_border_sample(border, border->line_anchor_x,
                             border->line_anchor_y);
      send_border_line(state, &ir, border->line_anchor_x, border->line_anchor_y,
                       border->line_theta);
      *mode = MODE_TURN_AWAY_FROM_LINE;
      send_status("line_angle_found_turning_away", state, true, *mode, &ir);
      start_turn(motion, state,
                 shortest_delta(state->theta,
                                normalize_angle(border->line_normal_theta +
                                                M_PI)),
                 TURN_PULSE_DELAY_TICKS);
      return;
    }

    if (border->align_steps >= MAX_ALIGN_STEPS) {
      send_status("align_timeout", state, false, *mode, &ir);
      *mode = MODE_IDLE;
      return;
    }

    border->align_steps++;
    start_turn(motion, state, (double)border->align_turn_sign * ALIGN_STEP_RAD,
               ALIGN_PULSE_DELAY_TICKS);
    return;

  case MODE_TURN_AWAY_FROM_LINE:
    *mode = MODE_LEAVE_LINE;
    send_status("leaving_line", state, true, *mode, &ir);
    start_straight(motion, state, LINE_CLEARANCE_STEP_M,
                   CLEARANCE_PULSE_DELAY_TICKS);
    return;

  case MODE_LEAVE_LINE:
    *mode = MODE_TURN_TO_LINE;
    send_status("line_clear_turning_right", state, true, *mode, &ir);
    start_turn(motion, state, shortest_delta(state->theta, border->follow_theta),
               TURN_PULSE_DELAY_TICKS);
    return;

  case MODE_TURN_TO_LINE:
    border->map_steps = 0U;
    border->line_distance_m = 0.0;
    *mode = MODE_BORDER_MAP;
    remember_border_sample(border, border->line_anchor_x,
                           border->line_anchor_y);
    send_border_point("map_start", state, &ir, border->line_anchor_x,
                      border->line_anchor_y, border->line_theta);
    send_status("mapping_border", state, false, *mode, &ir);
    return;

  case MODE_BORDER_MAP:
    if (any_ir_black(&ir) && border->map_steps >= MAP_CORNER_GRACE_STEPS) {
      double x = 0.0;
      double y = 0.0;
      send_sensor_border_point("corner_hit", state, &ir, state->theta, &x, &y);
      remember_border_sample(border, x, y);
      if (border_finished_by_geometry(border, x, y)) {
        send_status("border_path_revisited", state, false, MODE_IDLE, &ir);
        *mode = MODE_IDLE;
        return;
      }
      border->corner_count++;
      begin_alignment(mode, border, state, &ir);
      send_status("corner_aligning", state, false, *mode, &ir);
      return;
    }

    if (border->map_steps >= MAX_MAP_STEPS) {
      send_status("border_map_complete", state, false, MODE_IDLE, &ir);
      *mode = MODE_IDLE;
      return;
    }

    border->map_steps++;
    border->line_distance_m = (double)border->map_steps * MAP_STEP_M;
    border->total_mapped_distance_m += MAP_STEP_M;
    if (any_ir_black(&ir)) {
      double x = 0.0;
      double y = 0.0;
      send_sensor_border_point("map_tape", state, &ir, border->line_theta, &x,
                               &y);
      remember_border_sample(border, x, y);
      if (border_finished_by_geometry(border, x, y)) {
        send_status(border_loop_closed(border, x, y) ? "border_loop_closed"
                                                     : "border_path_revisited",
                    state, false, MODE_IDLE, &ir);
        *mode = MODE_IDLE;
        return;
      }
    } else {
      const double x = border->line_anchor_x +
                       (border->line_distance_m * cos(border->line_theta));
      const double y = border->line_anchor_y +
                       (border->line_distance_m * sin(border->line_theta));
      remember_border_sample(border, x, y);
      send_border_point(
          "map_point", state, &ir, x, y, border->line_theta);
      if (border_finished_by_geometry(border, x, y)) {
        send_status(border_loop_closed(border, x, y) ? "border_loop_closed"
                                                     : "border_path_revisited",
                    state, false, MODE_IDLE, &ir);
        *mode = MODE_IDLE;
        return;
      }
    }
    start_straight(motion, state, MAP_STEP_M, MAP_PULSE_DELAY_TICKS);
    return;
  }
}

static void apply_map_hint(const char *payload, map_hint_state_t *map_hint) {
  double value = 0.0;

  if (json_double_value(payload, "\"obstacleDistance\"", &value)) {
    map_hint->obstacle_distance_m = value;
  }
  if (json_double_value(payload, "\"obstacleBearing\"", &value)) {
    map_hint->obstacle_bearing_rad = value;
  }
  if (json_double_value(payload, "\"nearestBorderDistance\"", &value)) {
    map_hint->nearest_border_distance_m = value;
  }

  map_hint->received_at_ms = monotonic_ms();
  map_hint->valid = true;
}

static void configure_area_from_payload(const char *payload,
                                        area_state_t *area) {
  double value = 0.0;

  area->min_x = AREA_DEFAULT_MIN_X_M;
  area->max_x = AREA_DEFAULT_MAX_X_M;
  area->min_y = AREA_DEFAULT_MIN_Y_M;
  area->max_y = AREA_DEFAULT_MAX_Y_M;
  area->row_spacing_m = AREA_ROW_SPACING_M;
  area->sweep_step_m = AREA_SWEEP_STEP_M;
  area->sweep_theta = 0.0;

  if (json_double_value(payload, "\"areaMinX\"", &value)) {
    area->min_x = value;
  }
  if (json_double_value(payload, "\"areaMaxX\"", &value)) {
    area->max_x = value;
  }
  if (json_double_value(payload, "\"areaMinY\"", &value)) {
    area->min_y = value;
  }
  if (json_double_value(payload, "\"areaMaxY\"", &value)) {
    area->max_y = value;
  }
  if (json_double_value(payload, "\"rowSpacing\"", &value)) {
    area->row_spacing_m = value;
  }
  if (json_double_value(payload, "\"sweepStep\"", &value)) {
    area->sweep_step_m = value;
  }
  if (json_double_value(payload, "\"sweepTheta\"", &value)) {
    area->sweep_theta = normalize_angle(value);
  }
}

static void process_command(const char *payload, motion_state_t *motion,
                            border_state_t *border, area_state_t *area,
                            map_hint_state_t *map_hint,
                            telemetry_state_t *state, control_mode_t *mode,
                            vl53x *sensor, bool tof_ready) {
  const ir_readings_t ir = read_ir_sensors();

  printf(">> Incoming UART payload: %s\n", payload);
  fflush(stdout);

  if (command_is_map_hint(payload)) {
    apply_map_hint(payload, map_hint);
    return;
  }

  if (command_is_stop(payload)) {
    area->active = false;
    stop_motion(motion, state, mode, &ir);
    return;
  }

  if (command_is_mountain_start(payload)) {
    if (!tof_ready) {
      send_status("tof_unavailable", state, false, *mode, &ir);
      return;
    }

    reset_and_enable_motion();
    *mode = MODE_MOUNTAIN_DETECT;
    send_status("mode_mountain_detect", state, true, *mode, &ir);
    start_mountain_scan(motion, state, sensor);
    return;
  }

  if (command_is_border_start(payload)) {
    area->active = false;
    start_border_search(motion, border, state, mode, &ir);
    return;
  }

  if (command_is_area_start(payload)) {
    configure_area_from_payload(payload, area);
    start_area_sweep(motion, area, state, mode, &ir);
    return;
  }
}

static void poll_uart(uart_rx_t *rx, motion_state_t *motion,
                      border_state_t *border, area_state_t *area,
                      map_hint_state_t *map_hint, telemetry_state_t *state,
                      control_mode_t *mode, vl53x *sensor, bool tof_ready) {
  while (uart_has_data(UART0)) {
    const uint8_t byte = uart_recv(UART0);

    if (rx->length_index < sizeof(rx->length_bytes)) {
      rx->length_bytes[rx->length_index++] = byte;
      if (rx->length_index < sizeof(rx->length_bytes)) {
        continue;
      }

      rx->payload_length = read_u32_le(rx->length_bytes);
      rx->payload_index = 0U;

      if (rx->payload_length == 0U) {
        uart_rx_reset(rx);
      } else if (rx->payload_length > UART_PAYLOAD_MAX) {
        rx->discarding_payload = true;
      }
      continue;
    }

    if (rx->discarding_payload) {
      rx->payload_index++;
      if (rx->payload_index >= rx->payload_length) {
        printf("!! Discarded oversized UART payload (%u bytes)\n",
               rx->payload_length);
        fflush(stdout);
        uart_rx_reset(rx);
      }
      continue;
    }

    rx->payload[rx->payload_index++] = (char)byte;
    if (rx->payload_index >= rx->payload_length) {
      rx->payload[rx->payload_length] = '\0';
      process_command(rx->payload, motion, border, area, map_hint, state, mode,
                      sensor, tof_ready);
      uart_rx_reset(rx);
    }
  }
}

int main(void) {
  telemetry_state_t telemetry = {0};
  motion_state_t motion = {0};
  border_state_t border = {0};
  area_state_t area = {0};
  map_hint_state_t map_hint = {0};
  color_object_state_t color_object = {0};
  uart_rx_t uart_rx;
  vl53x sensor;
  bool tof_ready = false;
  control_mode_t mode = MODE_IDLE;
  uint64_t next_status_at_ms = 0U;

  uart_rx_reset(&uart_rx);
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  printf("VENUS experiment robot starting for module %s.\n", MODULE_NUMBER);
  printf("Challenge mode uses IR sensors on AR12/AR13 for border/crater detection.\n");
  printf("Color sensor uses AR4-AR8; ultrasonic trigger/echo use AR9/AR10.\n");
  printf("Send {\"cmd\":\"start_border\"} to search, align, turn, and map border tape.\n");
  printf("Area mode: server sends set_mode area_sweep with safe split bounds after border.\n");
  printf("Mountain mode: set_mode mountain_detect spins and scans ToF (%s).\n",
         HAVE_VL53L0X ? "compiled in" : "not compiled in");
  printf("Color detect: ultrasonic <= %.2f m, %.1f deg down at %.1f cm high, "
         "sample %u ms.\n",
         COLOR_TRIGGER_DISTANCE_M, ULTRASONIC_DOWN_ANGLE_DEG,
         ULTRASONIC_HEIGHT_M * 100.0, COLOR_SAMPLE_DURATION_MS);
  printf("Border traversal turn is fixed anticlockwise (sign %+d).\n",
         ANTICLOCKWISE_FOLLOW_TURN_SIGN);
  printf("Black detection level: %s\n",
         BLACK_DETECTED_LEVEL == GPIO_LEVEL_LOW ? "LOW" : "HIGH");
  fflush(stdout);

  pynq_init();
  switchbox_init();
  switchbox_set_pin(UART_RX_PIN, SWB_UART0_RX);
  switchbox_set_pin(UART_TX_PIN, SWB_UART0_TX);
  gpio_set_direction(LEFT_IR_PIN, GPIO_DIR_INPUT);
  gpio_set_direction(RIGHT_IR_PIN, GPIO_DIR_INPUT);
  uart_init(UART0);
  uart_reset_fifos(UART0);
  initialize_color_ultrasonic();
  initialize_motion();
  tof_ready = initialize_tof(&sensor);
  if (!tof_ready) {
    printf("ToF unavailable. Border mode still works; mountain mode reports tof_unavailable.\n");
    fflush(stdout);
  }

  next_status_at_ms = monotonic_ms();

  while (keep_running) {
    const uint64_t now_ms = monotonic_ms();

    poll_uart(&uart_rx, &motion, &border, &area, &map_hint, &telemetry, &mode,
              &sensor, tof_ready);
    poll_motion(&motion, &telemetry, &mode, &sensor);
    poll_color_object(&motion, &color_object, &area, &telemetry, &mode);
    poll_area_autonomy(&motion, &border, &area, &map_hint, &telemetry, &mode);
    poll_border_autonomy(&motion, &border, &telemetry, &mode);

    if (motion.telemetry_now || now_ms >= next_status_at_ms) {
      const ir_readings_t ir = read_ir_sensors();
      send_status(periodic_status(mode, &motion, tof_ready), &telemetry,
                  motion.active || mode == MODE_MOUNTAIN_DETECT, mode, &ir);
      motion.telemetry_now = false;
      next_status_at_ms = now_ms + STATUS_PERIOD_MS;
    }

    sleep_msec(POLL_DELAY_MS);
  }

  printf("Stopping VENUS experiment robot.\n");
  fflush(stdout);
  stepper_disable();
  stepper_destroy();
  pulsecounter_destroy(PULSECOUNTER0);
#if HAVE_VL53L0X
  iic_destroy(IIC0);
#endif
  uart_reset_fifos(UART0);
  uart_destroy(UART0);
  pynq_destroy();
  return EXIT_SUCCESS;
}
