#include <GyverPID.h>
#include <Bounce2.h>
#define methodWait 0
#define methodBlowUp 1
#define methodFireUp 2
#define methodWork 3
#define methodBlowDown 4
// * - переменную нельзя менять в объявлении
int candle_pin = 3;
int fuel_pump_pin = 5;
int fan_pin = 6;
int generator_sensor_pin = 8;
int temp_bool_sensor_pin = 9;
int temp2_sensor_pin = A5;
int temp_sensor_pin = A6;
int flame_sensor_pin = A7;
Bounce debouncer = Bounce(); //инициализация библиотеки антидребезга
GyverPID regulator(0.01, 0.01, 0, 1000); //инициализация библиотеки PID
//**************************Переменные внутри этого блока менять нельзя*********************//
float power = 0;              //*мощность топливного насоса
//таймеры 0-вкл. топливного насоса, 1-выкл. топл насоса,2-предпродувки,3-миним.предпродувки,4-поджига,
//5-перехода в рабочее состояние,6-миним.постпродукви,7-скор.регулировки мощности в процессе работы,
//8-скор.ввода принудительного запуска,9-проверка клика,10-частота отображения отладочной информации, 11- перезапуск счётчика неудачных запусков
unsigned long sMillis[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};            //*Объявление таймеров
unsigned long currentMillis = millis();   //*Объявление переменной "Тактирования"
int failStart = 3;
int fuelState = 0;    //*состояние топливного насоса HI/LOW
int currentMethod = 0;  //*текущий режим работы
float temp1 = 0;        //*переведённая температура с датчика1
float temp2 = 0;        //*переведённая температура с датчика2(дублирующего)
int fan_speed = 0;      //*текущая скорость вентилятора
float levelFlame = 0;     //*переведённое в попугаи значение датчика пламени
int onStartSensor = 1000;//*последние сохранённые данные с датчика пламени
int clicks = 0;     //*текущее количество переходов вкл/выкл датчика температуры
int lastClickState = 0;
//**************************КОНЕЦ БЛОКА НЕПРИКАСАЕМЫХ ПЕРЕМЕННЫХ***************************//
float intervals[15] = {0, 0.03, 0.02, 10, 1, 30, 60, 0.1, 1, 1, 1, 180};  // Интервалы работы таймеров(№см sMillis)
int max_fan_speed = 200;//максимальная скорость вентилятора
int min_fan_speed = 30;//максимальная скорость вентилятора
float min_temp = 30;    //температура запуска WEBASTO
float max_temp = 60;    //температура отключения WEBASTO
float min_power = 7.0;    //минимальное время подачи топлива
float max_power = 0.250;    //максимальное время подачи топлива
float flamePoint = 5;
int flameLevel[4] = {17, 21, 17, 21};
void setup() {
  Serial.begin(9600);
  pinMode(generator_sensor_pin, INPUT);
  pinMode(temp_bool_sensor_pin, INPUT);
  pinMode(flame_sensor_pin, INPUT);
  pinMode(temp_sensor_pin, INPUT);
  pinMode(temp2_sensor_pin, INPUT);
  pinMode(fuel_pump_pin, OUTPUT);
  pinMode(candle_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  pinMode(13, OUTPUT);
  debouncer.attach(temp_bool_sensor_pin);
  debouncer.interval(5); // interval in ms
  regulator.setDirection(REVERSE); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(90.0, 100.0);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = 10.0;        // сообщаем регулятору температуру, которую он должен поддерживать


  setTimer(10, intervals[10]); //запуск таймера вывода данных в ком-порт.
  setTimer(0, 1); //запуск таймера насоса
}

void loop() {
  currentMillis = millis();
  workers();
  fuel_pump();
  getSignal();
  digitalWrite(13, currentMethod == methodWork);
  if (currentMethod == methodWait) wait();
  else if (currentMethod == methodBlowUp) blowUp();
  else if (currentMethod == methodFireUp) fireUp();
  else if (currentMethod == methodBlowDown) blowDown();
  else if (currentMethod == methodWork) work();
  printDebug();
}

void blowUp() {
  if (checkTimer(2) == 2 || (checkTimer(2) == 1 && fan_speed < max_fan_speed)) {
    fan_speed++;
    setTimer(2, intervals[2]);
  }
  else if  (checkTimer(2) == 1 && fan_speed == max_fan_speed) {
    resetTimer(2);
    setTimer(3, intervals[3]);
  } else if (checkTimer(3) == 1) {
    currentMethod = methodFireUp;
    resetTimer(3);
  }
}

void blowDown() {
  power = 0;
  digitalWrite(candle_pin, LOW);
  if (checkTimer(6) == 2) setTimer(6, intervals[6]);
  if (levelFlame > flameLevel[3] && checkTimer(6) == 1) {
    currentMethod = methodWait;
    resetTimer(6);
  }
  if (fan_speed < max_fan_speed) fan_speed++;
}

void wait() {
  fan_speed = 0;
  onStartSensor = 1000;
  power = 0;
  digitalWrite(fuel_pump_pin, LOW);
  digitalWrite(candle_pin, LOW);
  //digitalWrite(fan_pin, LOW);
  if (failStart < 3 && digitalRead(generator_sensor_pin) == 0 &&
      (debouncer.read() == 0 && temp1 < min_temp && temp2 < min_temp) || (clicks == 5 && temp1 < max_temp && temp2 < max_temp))
    currentMethod = methodBlowUp;
  if (digitalRead(generator_sensor_pin) == 0 && levelFlame < flameLevel[0]) currentMethod = methodBlowDown;
  if (digitalRead(generator_sensor_pin) == 1) failStart = 0;
  if (checkTimer(11) == 2 && failStart != 0) setTimer(11, intervals[11]); else if (checkTimer(11) == 1) failStart = 0;
}

void workers() {
  temp1 = floatMap(analogRead(temp_sensor_pin), 0, 350, 15, 65.0);
  temp2 = floatMap(analogRead(temp2_sensor_pin), 0, 350, 15, 65.0);
  levelFlame = floatMap(analogRead(flame_sensor_pin), 370, 870, 0, 100);
  analogWrite(fan_pin, fan_speed);
  debouncer.update();
}

void fireUp() {
  if (checkTimer(5) == 1 && onStartSensor > analogRead(flame_sensor_pin)) {
    setTimer(5, intervals[5]);
    onStartSensor = analogRead(flame_sensor_pin);
  } else if (checkTimer(5) == 1) {
    currentMethod = methodBlowDown;
    failStart++;
    digitalWrite(candle_pin, LOW);
    resetTimer(4);
    return;
  }
  if (levelFlame < flameLevel[2]) {
    currentMethod = methodWork;
    digitalWrite(candle_pin, LOW);
    failStart = 0;
    resetTimer(4);
    resetTimer(5);
    setTimer(7, intervals[7]);
    resetTimer(11);
    return;
  }
  if (checkTimer(4) == 2) {
    onStartSensor = 1000;
    fan_speed = min_fan_speed;
    digitalWrite(candle_pin, HIGH);
    setTimer(5, intervals[5]);
    setTimer(4, intervals[4]);
  } else if (checkTimer(4) == 1) {
    if (fan_speed < max_fan_speed) fan_speed++;
    setTimer(4, intervals[4]);
    if (power < 100) power ++;
  }
}

void work() {
  regulator.input = levelFlame;
  if (temp1 > max_temp || temp2 > max_temp || digitalRead(generator_sensor_pin) == 1) currentMethod = methodBlowDown;
  if (fan_speed >= max_fan_speed && power >= 100) resetTimer(7);
  if (checkTimer(7) == 1) {
    if (fan_speed - 0.9 < max_fan_speed && fan_speed + 0.9 > max_fan_speed) fan_speed = max_fan_speed;
    if (fan_speed < max_fan_speed) fan_speed++;
    if (power - 1 < 100 && power + 1 > 100) power = 100;
    if (power < 100) power++;
    setTimer(7, intervals[7]);
  }

  //  if (checkTimer(9) == 2 && checkTimer(7) == 2) setTimer(9,intervals[9]);
  //  else if (checkTimer(9) == 1 && levelFlame < 3 && power > 80) {
  //    power = power - 0.1;
  //    setTimer(9,intervals[9]);
  //    } else if (checkTimer(9) == 1 && levelFlame > 10 && power < 100){
  //    power = power + 0.1;
  //    setTimer(9,intervals[9]);
  //    }

  if (checkTimer(9) == 2 && checkTimer(7) == 2) setTimer(9, intervals[9]);
  else if (checkTimer(9) == 1) {
    
    //power = regulator.getResultTimer();
    float error = (levelFlame - flamePoint) * 0.05;
    power = constrain(power + error, 90, 100);
    
    setTimer(9, intervals[9]);
  }

}

void printDebug() {
  if (checkTimer(10) == 1) {
    setTimer(10, 1);


    Serial.print("af:");
    Serial.print(analogRead(flame_sensor_pin));
    Serial.print(";");

    Serial.print("f:");
    Serial.print(levelFlame);
    Serial.print(";");

    Serial.print("t1:");
    Serial.print(temp1);
    Serial.print(";");

    Serial.print("t2:");
    Serial.print(temp2);
    Serial.print(";");

    Serial.print("p:");
    Serial.print(power);
    Serial.print(";");

    Serial.print("fan:");
    Serial.print(fan_speed);
    Serial.print(";");

    Serial.print("m:");
    Serial.print(currentMethod);
    Serial.print(";");

    Serial.print("c:");
    Serial.print(clicks);
    Serial.print(";");

    Serial.print("w:");
    Serial.print(millis() - currentMillis);
    Serial.print(";");

    Serial.println();
  }
}

void getSignal() {
  if (checkTimer(8) == 2 && debouncer.read() == 0) setTimer(8, intervals[8]);
  else if (checkTimer(8) == 0 && lastClickState != debouncer.read()) {
    clicks++;
    lastClickState = debouncer.read();
  }
  else if (checkTimer(8) == 1) {
    resetTimer(8);
    clicks = 0;
  }

}

void fuel_pump_min() {
  if (power == 0.0) {
    digitalWrite(fuel_pump_pin, LOW);
    return;
  }
  if (fuelState == 0) {
    if (checkTimer(0) == 1) {
      digitalWrite(fuel_pump_pin, HIGH);
      fuelState = 1;
      setTimer(1, intervals[1]);
      resetTimer(0);
    }
  }
  else {
    if (checkTimer(1) == 1) {
      digitalWrite(fuel_pump_pin, LOW);
      fuelState = 0;
      setTimer(0, power / 3);
      resetTimer(1);
    }
  }
}

void fuel_pump() {
  if (power == 0) {
    digitalWrite(fuel_pump_pin, LOW);
    return;
  }
  if (fuelState == 0) {
    if (checkTimer(0) == 1) {
      digitalWrite(fuel_pump_pin, HIGH);
      fuelState = 1;
      setTimer(1, intervals[1]);
      resetTimer(0);
    }
  }
  else {
    if (checkTimer(1) == 1) {
      digitalWrite(fuel_pump_pin, LOW);
      fuelState = 0;
      setTimer(0, floatMap(power, 0, 100, min_power, max_power));
      resetTimer(1);
    }
  }
}

void setTimer(int index, float sec) {
  long timed = sec * 1000;
  sMillis[index] = currentMillis + timed;
}

void resetTimer(int index) {
  sMillis [index] = 0;
}

int checkTimer(int index) {
  if (sMillis[index] == 0) return 2;
  if (sMillis[index] <= currentMillis) {
    return 1;
  }
  return 0;
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
