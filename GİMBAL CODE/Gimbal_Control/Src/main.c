#include <stdint.h>
#include <math.h>

#define PERIPH_BASE           ((uint32_t)0x40000000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000)
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)

#define RCC_BASE              (PERIPH_BASE + 0x00021000)
#define RCC_AHB2ENR           *(volatile uint32_t *)(RCC_BASE + 0x4C)
#define RCC_APB1ENR1          *(volatile uint32_t *)(RCC_BASE + 0x58)
#define RCC_APB2ENR           *(volatile uint32_t *)(RCC_BASE + 0x60)

#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define TIM1_CR1              *(volatile uint32_t *)(TIM1_BASE + 0x00)
#define TIM1_CCR1             *(volatile uint32_t *)(TIM1_BASE + 0x34)
#define TIM1_CCR2             *(volatile uint32_t *)(TIM1_BASE + 0x38)
#define TIM1_CCR3             *(volatile uint32_t *)(TIM1_BASE + 0x3C)

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
} PID_Controller;

PID_Controller pitch_pid = {.Kp = 2.5f, .Ki = 0.05f, .Kd = 0.8f, .prev_error = 0.0f, .integral = 0.0f};

float target_angle = 0.0f;   
float current_angle = 0.0f;  
float dt = 0.01f;     

float Complementary_Filter(float accel_angle, float gyro_rate, float loop_dt) {
    float alpha = 0.98f; 
    return alpha * (current_angle + gyro_rate * loop_dt) + (1.0f - alpha) * accel_angle;
}

float PID_Compute(PID_Controller *pid, float setpoint, float measured_value, float loop_dt) {
    float error = setpoint - measured_value;
    
    float p_out = pid->Kp * error;

    pid->integral += error * loop_dt;
    float i_out = pid->Ki * pid->integral;

    float derivative = (error - pid->prev_error) / loop_dt;
    float d_out = pid->Kd * derivative;
    
    pid->prev_error = error;
    
    return p_out + i_out + d_out;
}

void Set_Motor_PWM(uint16_t u, uint16_t v, uint16_t w) {
    TIM1_CCR1 = u;
    TIM1_CCR2 = v;
    TIM1_CCR3 = w;
}

int main(void) {
    float raw_accel_angle = 5.2f; 
    float raw_gyro_rate = -0.1f;

    while (1) {
  
        current_angle = Complementary_Filter(raw_accel_angle, raw_gyro_rate, dt);

        float pid_output = PID_Compute(&pitch_pid, target_angle, current_angle, dt);
        
        uint16_t pwm_u = (uint16_t)(500 + pid_output);
        uint16_t pwm_v = (uint16_t)(500 - pid_output);
        uint16_t pwm_w = 500;
        
        Set_Motor_PWM(pwm_u, pwm_v, pwm_w);
        
        for (volatile uint32_t i = 0; i < 100000; i++);
    }

    return 0;
}