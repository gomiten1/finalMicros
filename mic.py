from machine import Pin, PWM, I2C, UART, Timer
import utime
import ustruct
from i2c_lcd import I2cLcd


# --- Ultrasonic Sensor ---
pin_trigger = Pin(2, Pin.OUT)
pin_echo = Pin(3, Pin.IN)
pin_trigger.low()

# --- LCD ---
i2c_lcd_bus = I2C(0, scl=Pin(5), sda=Pin(4))
lcd = I2cLcd(i2c_lcd_bus, 0x27, 2, 16)

# --- Servos ---
ud_virtual_pos = 0
UD_LIMIT_MAX = 8   
UD_LIMIT_MIN = -8  
PIN_SERVO_UP_DOWN = 6 
PIN_SERVO_LEFT_RIGHT = 7 
servo_ud = PWM(Pin(PIN_SERVO_UP_DOWN))
servo_lr = PWM(Pin(PIN_SERVO_LEFT_RIGHT))
servo_ud.freq(50)
servo_lr.freq(50)

# --- Accelerometer ---
i2c_accel = I2C(1, scl=Pin(11), sda=Pin(10), freq=400000)
MPU_ADDR = 0x68
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_START = 0x3B
accel_ready = False

# --- Laser ---
pin_laser = Pin(13, Pin.OUT)
pin_laser.low()

# --- 7 Seg ---
COMMON_CATHODE = True
PINS_7SEG = [16, 17, 18, 19, 20, 21, 22]
segments = [Pin(p, Pin.OUT) for p in PINS_7SEG]
DIGIT_MAP = {
    0: [1, 1, 1, 1, 1, 1, 0],
    1: [0, 0, 0, 0, 1, 1, 0],
    2: [1, 1, 0, 1, 1, 0, 1],
    3: [1, 0, 0, 1, 1, 1, 1]
}

# --- Push ---
btn_trigger = Pin(27, Pin.IN, Pin.PULL_UP)

# --- Potentiometer ---
pin_pot_reset = Pin(26, Pin.IN)

# --- Buzzer ---
buzzer = PWM(Pin(28))
buzzer.duty_u16(0)

STATE_IDLE = 0
STATE_ALARM_FLASH = 1
STATE_ALARM_SEARCH = 2
STATE_ATTACK = 3

current_state = STATE_IDLE
attack_flag_irq = False
target_reset_state = 0

angle_ud = 90
angle_lr = 90
SERVO_STEP = 12

state_start_time = 0
sweep_direction = -1
last_sweep_update = 0

# FUNCTIONS

def pad16(text):
    text = str(text)
    if len(text) > 16:
        return text[:16]
    return text + " " * (16 - len(text))

def update_lcd(line1, line2):
    try:
        lcd.move_to(0, 0)
        lcd.putstr(pad16(line1))
        lcd.move_to(0, 1)
        lcd.putstr(pad16(line2))
    except Exception as e:
        pass

def show_digit(num):
    if num not in DIGIT_MAP:
        state_off = 0 if COMMON_CATHODE else 1
        for seg in segments:
            seg.value(state_off)
        return
    bits = DIGIT_MAP[num]
    for i in range(7):
        val = bits[i] if COMMON_CATHODE else (1 - bits[i])
        segments[i].value(val)

def move_servo(servo, angle):
    angle = max(0, min(180, angle))
    pulse_ns = int(500000 + (angle / 180.0) * 2000000)
    servo.duty_ns(pulse_ns)

def init_mpu6050():
    global accel_ready
    try:
        i2c_accel.writeto_mem(MPU_ADDR, REG_PWR_MGMT_1, bytes([0x00]))
        utime.sleep_ms(100)
        accel_ready = True
    except:
        accel_ready = False

def get_accel_data():
    if not accel_ready:
        return (0.0, 0.0, 0.0)
    try:
        data = i2c_accel.readfrom_mem(MPU_ADDR, REG_ACCEL_START, 6)
        x, y, z = ustruct.unpack('>hhh', data)
        return (x/16384.0, y/16384.0, z/16384.0)
    except:
        return (0.0, 0.0, 0.0)

def detect_movement(threshold=0.5):
    ax1, ay1, az1 = get_accel_data()
    utime.sleep_ms(20)
    ax2, ay2, az2 = get_accel_data()
    if abs(ax2-ax1) > threshold or abs(ay2-ay1) > threshold or abs(az2-az1) > threshold:
        return True
    return False

def get_distance_cm():
    pin_trigger.low()
    utime.sleep_us(2)
    pin_trigger.high()
    utime.sleep_us(10)
    pin_trigger.low()
    
    timeout_us = 30000
    start = utime.ticks_us()
    while pin_echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout_us:
            return -1.0
            
    t1 = utime.ticks_us()
    while pin_echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), t1) > timeout_us:
            return -1.0
            
    t2 = utime.ticks_us()
    duration = utime.ticks_diff(t2, t1)
    dist = (duration * 0.0343) / 2
    return dist if 2.0 <= dist <= 400.0 else -1.0

def process_bluetooth():
    global angle_ud, angle_lr, ud_virtual_pos
    if uart_bt.any():
        try:
            cmd = uart_bt.read(1).decode('utf-8').strip().upper()
            if not cmd: return False
            
            moved = False
            if cmd == 'U':
                if ud_virtual_pos < UD_LIMIT_MAX:          
                    ud_virtual_pos += 1
                    move_servo(servo_ud, 125)   
                    utime.sleep_ms(50)          
                    angle_ud = 90               
                    moved = True
            elif cmd == 'D':
                if ud_virtual_pos > UD_LIMIT_MIN:         
                    ud_virtual_pos -= 1
                    move_servo(servo_ud, 55)    
                    utime.sleep_ms(50)          
                    angle_ud = 90               
                    moved = True
            elif cmd == 'L':
                angle_lr = min(180, angle_lr + SERVO_STEP)
                moved = True
            elif cmd == 'R':
                angle_lr = max(0, angle_lr - SERVO_STEP)
                moved = True
            elif cmd == 'S':
                ud_virtual_pos = 0              
                angle_ud, angle_lr = 90, 90
                moved = True
                
            if moved:
                move_servo(servo_ud, angle_ud)
                move_servo(servo_lr, angle_lr)
            return moved
        except:
            return False
    return False

def trigger_buzzer(active, freq=1000):
    if active:
        buzzer.freq(freq)
        buzzer.duty_u16(32768)
    else:
        buzzer.duty_u16(0)

def song():
    melody = [
        (523, 120),  # C5
        (659, 120),  # E5
        (784, 180),  # G5
        (659, 120),  # E5
    ]
    
    for freq, duration in melody:
        buzzer.freq(freq)
        buzzer.duty_u16(45000)
        utime.sleep_ms(duration)
        buzzer.duty_u16(0)
        utime.sleep_ms(40)

# 4. ISR

def button_isr(pin):
    global attack_flag_irq
    attack_flag_irq = True

btn_trigger.irq(trigger=Pin.IRQ_FALLING, handler=button_isr)

# 5. FSM LOOP

def main():
    global current_state, state_start_time, angle_lr, angle_ud
    global attack_flag_irq, sweep_direction, last_sweep_update
    global target_reset_state, ud_virtual_pos
    
    init_mpu6050()
    move_servo(servo_ud, 90)
    move_servo(servo_lr, 90)
    
    # Check initial position of the potentiometer to arm the reset logic
    # If it starts near 0V, next trigger is 1 (3.3V). If 1, next is 0.
    target_reset_state = 1 if pin_pot_reset.value() == 0 else 0
    
    while True:
        now = utime.ticks_ms()
        
        # --- HARDWARE RESET OVERRIDE ---
        if pin_pot_reset.value() == target_reset_state:
            current_state = STATE_IDLE
            attack_flag_irq = False
            pin_laser.low()
            trigger_buzzer(False)
            
            # Reset hardware positions
            ud_virtual_pos = 0
            angle_lr, angle_ud = 90, 90
            move_servo(servo_ud, 90)
            move_servo(servo_lr, 90)
            
            # Swap target for the next toggle
            target_reset_state = 1 if target_reset_state == 0 else 0
            
            update_lcd("SYSTEM RESET", "Hardware Override")
            utime.sleep_ms(800) # Give user time to let go of the pot
            continue
        
        # --- STATE: IDLE ---
        if current_state == STATE_IDLE:
            show_digit(0)
            update_lcd("State: IDLE", "System OK")
            pin_laser.low()
            trigger_buzzer(False)
            
            # 1. Check BT
            process_bluetooth()
            
            # 2. Check Attack Button
            if attack_flag_irq:
                attack_flag_irq = False
                current_state = STATE_ATTACK
                state_start_time = now
                continue
                
            # 3. Check Ultrasonic
            dist = get_distance_cm()
            if dist != -1.0 and dist < 6.0:
                current_state = STATE_ALARM_FLASH
                state_start_time = now
                continue
                
            # 4. Check Accelerometer
            if detect_movement():
                current_state = STATE_ALARM_SEARCH
                angle_lr = 180 
                angle_ud = 90  
                sweep_direction = -1
                move_servo(servo_lr, angle_lr)
                move_servo(servo_ud, angle_ud)
                state_start_time = now
                last_sweep_update = now
                continue

        # --- STATE: ALARM_FLASH ---
        elif current_state == STATE_ALARM_FLASH:
            show_digit(1)
            update_lcd("Intruder Alert", "FLASHING")
            
            elapsed = utime.ticks_diff(now, state_start_time)
            if (elapsed // 100) % 2 == 0:
                pin_laser.high()
                trigger_buzzer(True, 1200)
            else:
                pin_laser.low()
                trigger_buzzer(True, 800) 
                
            if elapsed > 5000:
                current_state = STATE_IDLE

        # --- STATE: ALARM_SEARCH ---
        elif current_state == STATE_ALARM_SEARCH:
            show_digit(2)
            update_lcd("Searching...", "Movement Det")
            
            # 1. Check ultrasonic sensor during sweep
            dist = get_distance_cm()
            if dist != -1.0 and dist < 6.0:
                current_state = STATE_ALARM_FLASH
                state_start_time = now
                pin_laser.low()
                trigger_buzzer(False)
                continue
            
            # 2. Non-blocking servo sweep logic
            if utime.ticks_diff(now, last_sweep_update) > 20: 
                angle_lr += sweep_direction * 2
                move_servo(servo_lr, angle_lr)
                last_sweep_update = now
                
                # Reverse sweep or exit
                if angle_lr <= 0:
                    current_state = STATE_IDLE
                    angle_ud, angle_lr = 90, 90
                    move_servo(servo_lr, 90)
                    move_servo(servo_ud, 90)
            
            # Keep laser and buzzer off during search
            pin_laser.low()
            trigger_buzzer(False)

        # --- STATE: ATTACK ---
        elif current_state == STATE_ATTACK:
            
                
            update_lcd("Mode: ATTACK", "Target Locked")
            
            elapsed = utime.ticks_diff(now, state_start_time)
            
                
            
            # Phase 1: Countdown (0 to 3 seconds)
            if elapsed < 1000:
                show_digit(3)
                song()
                pin_laser.low()
            elif elapsed < 2000:
                show_digit(2)
                pin_laser.low()
            elif elapsed < 3000:
                show_digit(1)
                pin_laser.low()
            # Phase 2: Attack Flashing + Small Up/Down Movement (3 to 6 seconds)
            else:
                show_digit(99)
                
                # Flashing logic
                if (elapsed // 80) % 2 == 0:
                    pin_laser.high()
                else:
                    pin_laser.low()
                    
                # UP DOWN movement
                cycle_time = (elapsed - 3000) % 800
                
                if cycle_time < 50:
                    move_servo(servo_ud, 105) # 50ms nudge UP
                elif cycle_time < 400:
                    move_servo(servo_ud, 90)  # Static wait
                elif cycle_time < 450:
                    move_servo(servo_ud, 60)  # 50ms nudge DOWN
                else:
                    move_servo(servo_ud, 90)  # Static wait            
                       
                
            # Exit condition: 6 seconds total elapsed
            if elapsed > 6000:
                current_state = STATE_IDLE
                attack_flag_irq = False
                move_servo(servo_ud, 90)

if __name__ == '__main__':
    main()
