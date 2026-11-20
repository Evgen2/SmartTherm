/* MyWiFiClient.hpp */

#include <WiFi.h>
#include <lwip/sockets.h>
#include <ArduinoHA.h>

#define DEBUG_CUSTOM_WIFI 0
#define WRITE_MAX_DURATION_MS 10000 // Максимум  мсекунд на отправку одного пакета
#define SELECT_STEP_US 50000       // Шаг ожидания 50 мс
#define CALLBACK_INTERVAL_MS 500   // Интервал вызова callback_loop при задержке 

class CustomWiFiClient : public WiFiClient {
public:
  using WiFiClient::WiFiClient;
#if DEBUG_CUSTOM_WIFI  
 // Переопределяем stop(), чтобы ловить ЛЮБОЕ закрытие сокета (и из write, и из PubSubClient)
  void stop() override {
    printTimestamp();
    Serial.println(" [CustomWiFiClient] Вызван stop()!");
    WiFiClient::stop();
  }

  // Перегруженный внутренний метод с указанием причины для отладки внутри write()
  void stopWithReason(const char* reason) {
    printTimestamp();
    Serial.printf(" [CustomWiFiClient] Вызван stop()! Причина: %s\n", reason);
    WiFiClient::stop();
  }
#endif //DEBUG_CUSTOM_WIFI

  void (*callback_loop)(int src) = nullptr; // Указатель на функцию обратного вызова

  // Метод для привязки функции
  void setLoopCallback(void (*cb)(int src)) {
    callback_loop = cb;
  }

  size_t write(uint8_t b) override {
    return write(&b, 1);
  }

  size_t write(const uint8_t *buf, size_t size) override {
    int socketFileDescriptor = fd();
    if (!connected() || (socketFileDescriptor < 0) || size == 0) {
      return 0;
    }

    size_t totalBytesSent = 0;
    size_t bytesRemaining = size;
    unsigned long startTime = millis();
    unsigned long lastCallbackTime = startTime; // Таймер для вызова callback_loop    

#if DEBUG_CUSTOM_WIFI
    int nretrys=0;
    int nretrys1=0;
    unsigned long t0 = micros();
#endif

    while (bytesRemaining > 0) {
      // Проверяем, не вышли ли мы за общий лимит времени 
      if (millis() - startTime >= WRITE_MAX_DURATION_MS) {
        // За WRITE_MAX_DURATION_MS сеть не приняла пакет — разрываем зависший сокет
#if DEBUG_CUSTOM_WIFI
      Serial.printf("[MQTT Client] Timeout: %u  %d %d, connected %d, dt %ld\n", totalBytesSent,  nretrys, nretrys1, connected(), micros()-t0);
        stopWithReason("Превышен дедлайн WRITE_MAX_DURATION_MS");        
#endif        
        stop();
        return totalBytesSent;
      }

  //  Проверяем, не пора ли вызвать callback_loop
      checkAndRunCallback(lastCallbackTime, 0x11);  

      fd_set set;
      struct timeval tv;
      FD_ZERO(&set);
      FD_SET(socketFileDescriptor, &set);
      tv.tv_sec = 0;
      tv.tv_usec = SELECT_STEP_US;
#if  DEBUG_CUSTOM_WIFI
        nretrys1++;
#endif
      int sel = select(socketFileDescriptor + 1, NULL, &set, NULL, &tv);
      if (sel < 0) {
        stop();
        // Ошибка сокета — закрываем соединение
#if DEBUG_CUSTOM_WIFI        
       Serial.printf("[MQTT Client] Ошибка select: errno=%d (%s)\n", errno, strerror(errno));        
            stopWithReason("Ошибка select < 0");
#endif            
        return 0;
      }
      if (sel == 0) {
        // Таймаут SELECT_STEP_US: буфер занят, отдаем время FreeRTOS и пробуем снова
        delay(1);
        continue;
      }

      if (FD_ISSET(socketFileDescriptor, &set)) {
#if DEBUG_CUSTOM_WIFI
          nretrys++;
#endif          
        int res = send(socketFileDescriptor, (void*)buf, bytesRemaining, MSG_DONTWAIT);
        if (res > 0) 
        {
          totalBytesSent += res;
          buf += res;
          bytesRemaining -= res;
          // Если ушла хотя бы часть данных, продлеваем дедлайн, так как связь жива
          startTime = millis();
          //todo
        } else if (res < 0) {
          if (errno != EAGAIN && errno != EWOULDBLOCK) {
#if DEBUG_CUSTOM_WIFI            
            char str[80];
            sprintf(str, "Ошибка errno %d", errno);
            stopWithReason(str);
#endif            
            stop();
            return 0;
          }
          //delay(1);
        } else {
          //delay(1);
        }

      }
    }

#if DEBUG_CUSTOM_WIFI

    if (totalBytesSent < size) {
      Serial.printf("[MQTT Client] Внимание: отправлено %u из %u байт nretrys %d %d\n", totalBytesSent, size, nretrys, nretrys1);
    } else {
      if(micros()-t0 > 100000)
      Serial.printf("%u  %d %d, dt %ld\n", totalBytesSent,  nretrys, nretrys1, micros()-t0);
    }
#endif // DEBUG_CUSTOM_WIFI
    
    return totalBytesSent;
  }
private:
  // Вспомогательный метод проверки времени и сброса счетчика
  inline void checkAndRunCallback(unsigned long &lastTime, int src) {
    if (callback_loop != nullptr && (millis() - lastTime >= CALLBACK_INTERVAL_MS)) {
      callback_loop(src);
      lastTime = millis(); // Сбрасываем счетчик после выполнения функции
    }
  }  

  #if DEBUG_CUSTOM_WIFI
 // Вспомогательная функция печати времени (работает, если настроен NTP/время, иначе выводит аптайм)
  void printTimestamp() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10)) { // Ждем не более 10 мс
      Serial.printf("%04d-%02d-%02d %02d:%02d:%02d",
                    timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
      // Если NTP еще не синхронизирован, выводим время с момента старта в секундах
      unsigned long sec = millis() / 1000;
      Serial.printf("[Uptime %02lu:%02lu:%02lu]", sec / 3600, (sec % 3600) / 60, sec % 60);
    }
  }  
#endif //DEBUG_CUSTOM_WIFI  
};
