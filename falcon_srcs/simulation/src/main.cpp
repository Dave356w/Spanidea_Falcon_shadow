#include "main.h"
#include "movement_service.h"

#define TIME_SCALE_VAL 1

// #define TEMP_WAIT (100 / TIME_SCALE_VAL)
#define TEMP_INIT_WAIT (1000000 / TIME_SCALE_VAL)
#define TIMER_INTERVAL ((20 * 1000 * 1000) / TIME_SCALE_VAL)
// #define VEL_THRESHOLD_SET_TIMEOUT (200 / TIME_SCALE_VAL)


RollingAvg<uint32_t> x_acc_avg(64);
RollingAvg<float> pressure_avg_g(32);
RollingAvg<float> acceleration_avg(16);
RollingAvg<float> adj_acc_avg_g(16);

// temporary varaibles for logging need to remove
uint16_t x_axis_1, x_axis_2, y_axis_1, y_axis_2, z_axis_1, z_axis_2; 
uint8_t sensor_value_updated = 0;

float pressure_g = 0.0, acc_mss_g = 0.0, vel_ms_g = 0.0;
float  adj_acc_g = 0.0, vel_threshold_g = 0.0;
SystemStates state = SystemStates::SYSTEM_STATE_INITIALIZING;
MotionStates motion_state = MotionStates::NOT_MOVING;
MonitorStates monitor_state = MonitorStates::NOT_MONITORING;

uint32_t temp_timer = 0;
FILE *fd;
uint16_t dec_counter = 0;
float prev_acc_avg = 0.0;
uint32_t init_time_g = 0;

uint8_t diff_acc_counter = 0;
float vel_threshold_mul_g = 5;
float diff_acc_mss_g = 0.0;
uint8_t alarm_status = 0;
float variance = 0.0;


MovementService ms(&acceleration_avg, &acc_mss_g, &adj_acc_g, &vel_ms_g, &pressure_avg_g);



void initialization()
{
  char line[500];
  
  for (int i = 0; i < 15; i++) {
    if (fgets(line, 500, fd) == NULL) {
      exit(1);
      return ;
    }
  }
  

  z_axis_1 = atoi(strtok(line, ","));
  z_axis_2 = atoi(strtok(NULL, ","));
  x_acc_avg.fill(((z_axis_1+z_axis_2) / 2));
  acc_mss_g = atof(strtok(NULL, ","));
  acceleration_avg.fill(atof(strtok(NULL, ",")));
  adj_acc_g = atof(strtok(NULL, ","));

  (void)strtok(NULL, ",");
  (void)strtok(NULL, ",");
  (void)strtok(NULL, ",");
  (void)strtok(NULL, ",");
  (void)strtok(NULL, ",");
  adj_acc_avg_g.fill(acceleration_avg.avg());
  char *ttt = strtok(NULL, ",");
  pressure_avg_g.fill((float)atof(ttt) / 10.0);

  init_time_g = millis();
  // ms.vel_threshold_adj = 5;
  state = SystemStates::SYSTEM_STATE_NOMINAL;
}

void alarm_service()
{
  alarm_status = 1;
    // nothing
}
void enable_alarm()
{
  if (alarm_status == 1) {
    return ;
  }

  alarm_status = 1;
}
void disable_alarm()
{
  if (alarm_status == 0) {
    return ;
  }
  alarm_status = 0;
  // nothing
}

void loop() {
  if (sensor_value_updated) {
    char log_con[500];
    sprintf(log_con, 
            "date, %d, %d, %f, %f, %f, %f, %f, %d, %d, %10lf, %f, %f", 
            z_axis_1, z_axis_2, acc_mss_g, acceleration_avg.avg(), adj_acc_g, vel_ms_g, ms.vel_threshold, ms.state, alarm_status, ms.variance_acc, pressure_avg_g.avg(), ms.variance_pres);
    
    printf("%s\n", log_con);

    sensor_value_updated = 0;

    switch (state) {
      case SystemStates::SYSTEM_STATE_INITIALIZING:
        initialization();
        break;
      case SystemStates::SYSTEM_STATE_NOMINAL:
        // if (init_time_g > 0 && (millis() - init_time_g) > 60000) {
        //   // ms.vel_threshold_adj = 1.5;
        //   init_time_g = 0;
        // }
        ms.run();
        break;
      default:
        state = SystemStates::SYSTEM_STATE_HOLD;
        break;
    }
  }

}


// read z-axis accelerometer data and convert to m/(s*s)
float read_acceleration_mss()
{
    char line[500];
    if (fgets(line, 500, fd) == NULL) {
        exit(1);
        return -1;
    }
    
    // char *test = NULL;
    // test = strtok(line, ",");
    // printf("test: %s\n", test);

    z_axis_1 = atoi(strtok(line, ","));
    z_axis_2 = atoi(strtok(NULL, ","));
    x_acc_avg.add(((z_axis_1+z_axis_2) / 2));
    float acc_mss = (x_acc_avg.avg() * 0.001220703) - 2.5;
    acc_mss = (acc_mss * 9.80665);
    if (acc_mss > 0) {
      acc_mss -= 9.80665;
    } else {
      acc_mss += 9.80665;
    }

    {
      (void)strtok(NULL, ",");
      (void)strtok(NULL, ",");
      (void)strtok(NULL, ",");
      (void)strtok(NULL, ",");
      (void)strtok(NULL, ",");
      (void)strtok(NULL, ",");
      (void)strtok(NULL, ",");
      (void)strtok(NULL, ",");
      char *test = strtok(NULL, ",");
      pressure_avg_g.add((float)atof(test) / 10.0);
    }

    return acc_mss;
}

void enable_timer1()
{
  // nothing
}

void disable_timer1()
{
  // reset Control Register to disable timer
}



void *thread_timer_fn(void *arg)
{
    struct timespec tr;

    if (clock_gettime(CLOCK_MONOTONIC, &tr) < 0) {
        pthread_exit(NULL);
    }

    uint64_t time_int = tr.tv_sec * 1000000000L + tr.tv_nsec;
    uint32_t interv_time_ns = TIMER_INTERVAL;
    long next_tick;

    while (1) {
        acc_mss_g = (read_acceleration_mss());// + acc_mss_g) / 2;
        acceleration_avg.add(acc_mss_g + adj_acc_g);

        if (ms.state >=  MotionStates::MOVEMENT_DETECTED && ms.state < MotionStates::DECELERATING) {
          vel_ms_g = vel_ms_g + (acceleration_avg.avg()  * 0.02);
        }
        if ((ms.state == MotionStates::NOT_MOVING || ms.state == MotionStates::ERROR_RESET)  && fabs(acceleration_avg.avg()) < 0.04) {
          adj_acc_avg_g.add(acceleration_avg.avg());

          if (adj_acc_avg_g.get(0) == adj_acc_avg_g.get(adj_acc_avg_g.size() - 1)) {
            adj_acc_g = acc_mss_g * -1;
          }
        }

        sensor_value_updated = 1;


        next_tick = (tr.tv_sec * 1000000000L + tr.tv_nsec) + interv_time_ns;
        tr.tv_sec = next_tick / 1000000000L;
        tr.tv_nsec = next_tick % 1000000000L;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tr, NULL);
    }
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
      printf("Please provide log file name!\n");
      return -1;
    }

    fd = fopen(argv[1], "r");
    // fd_log = fopen("dummy.log", "r+");
    initialization();
    pthread_t thread_timer;

    int thread_timer_id = pthread_create(&thread_timer, NULL, thread_timer_fn, NULL);

    while(1) {
        loop();
        // usleep(1);
    }
    pthread_join(thread_timer, NULL);

    
    return 0;
}