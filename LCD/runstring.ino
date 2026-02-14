#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Функция для двух бегущих строк одновременно
void dualScrollingText(String text1, String text2, int speedDelay) {
  int len1 = text1.length();
  int len2 = text2.length();
  
  // Цикл прокрутки для обеих строк
  for (int pos1 = 16 + len1; pos1 >= -len1; pos1--) {
    lcd.clear();
    
    // Первая строка
    lcd.setCursor(pos1, 0);
    lcd.print(text1);
    
    // Вторая строка  
    lcd.setCursor(pos1, 1);
    lcd.print(text2);
    
    delay(speedDelay);
  }
}

void setup() {
  lcd.begin(16, 2);
}

void loop() {
  // Разные тексты на каждой строке
  dualScrollingText("PLOTINA  LOGI", "CLUB CLUB CLUB", 300);
  
  delay(1000);  // Пауза перед повтором
}
