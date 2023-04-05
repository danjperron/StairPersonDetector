#define FSAMP 200
#define NSAMP 256


uint8_t mpu6050_status();
void mpu6050_reset();
void mpu6050_read_acc(int16_t accel[3]);


