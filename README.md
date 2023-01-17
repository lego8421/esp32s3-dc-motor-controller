# ESP32S3 DC Motor Controller

## AT-Command

AT 커맨드를 사용하여, 파라미터 수정

* Serialport(UART), baud: 115200

### AT+CUR

current control

```
AT+CUR?

AT+CUR=TAR_CUR
```

TAR_CUR: 목표 전류[A]

* Example
``` bash
# 모터 전류 받아오기
> AT+CUR?
read current: 0.02A

# 모터 목표 전류를 0.5A로 설정
> AT+CUR=0.5
write current: 0.5A
```

### AT+VEL

velocity control

```
AT+VEL?

AT+VEL=TAR_VEL
```

TAR_VEL: 목표 속도[rad/s]

* Example
``` bash
# 모터 속도 받아오기
> AT+VEL?
read velocity: 0.02A

# 모터 목표 속도를 0.5rad/s로 설정
> AT+VEL=0.5
write velocity: 0.5rad/s
```

### AT+POS

position control

```
AT+POS?

AT+POS=TAR_POS
```

TAR_POS: 목표 위치[rad]

* Example
``` bash
# 모터 위치 받아오기
> AT+POS?
read position: 3.141rad

# 모터 목표 위치를 6.283rad로 설정
> AT+POS=6.283
write position: 6.283rad
```
