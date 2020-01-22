#include "Encoder.h"

Encoder::Encoder(int pa, int pb) {
  pinA = pa;
  pinB = pb;
}

void Encoder::init(int i1, int i2) {
  pinMode(pinA, INPUT); // Пины в режим приема INPUT
  pinMode(pinB, INPUT); // Пины в режим приема INPUT

  attachInterrupt(i1, A, CHANGE);
  attachInterrupt(i2, B, CHANGE);
}

static void Encoder::A() {
  if (micros() - lastTurn < pause)
    return; // Если с момента последнего изменения состояния не прошло
  // достаточно времени - выходим из прерывания
  pinAValue = digitalRead(pinA); // Получаем состояние пинов A и B
  pinBValue = digitalRead(pinB);

  cli(); // Запрещаем обработку прерываний, чтобы не отвлекаться
  if (state == 0 && !pinAValue && pinBValue ||
      state == 2 && pinAValue && !pinBValue) {
    state += 1; // Если выполняется условие, наращиваем переменную state
    lastTurn = micros();
  }
  if (state == -1 && !pinAValue && !pinBValue ||
      state == -3 && pinAValue && pinBValue) {
    state -= 1; // Если выполняется условие, наращиваем в минус переменную state
    lastTurn = micros();
  }
  setCount(state); // Проверяем не было ли полного шага из 4 изменений сигналов
                   // (2 импульсов)
  sei();           // Разрешаем обработку прерываний

  if (pinAValue && pinBValue && state != 0)
    state = 0;
}

static void Encoder::B() {
  if (micros() - lastTurn < pause)
    return;
  pinAValue = digitalRead(pinA);
  pinBValue = digitalRead(pinB);

  cli();
  if (state == 1 && !pinAValue && !pinBValue ||
      state == 3 && pinAValue && pinBValue) {
    state += 1; // Если выполняется условие, наращиваем переменную state
    lastTurn = micros();
  }
  if (state == 0 && pinAValue && !pinBValue ||
      state == -2 && !pinAValue && pinBValue) {
    state -= 1; // Если выполняется условие, наращиваем в минус переменную state
    lastTurn = micros();
  }
  setCount(state); // Проверяем не было ли полного шага из 4 изменений сигналов
                   // (2 импульсов)
  sei();

  if (pinAValue && pinBValue && state != 0)
    state = 0;
}

void Encoder::setCount(int state) { // Устанавливаем значение счетчика
  if (state == 4 || state == -4) {
    count += (int)(state / 4); // Увеличиваем/уменьшаем счетчик
    lastTurn = micros(); // Запоминаем последнее изменение
  }
}

void Encoder::reset() {}

long Encoder::get() {
  actualcount = count;
  return actualcount;
}