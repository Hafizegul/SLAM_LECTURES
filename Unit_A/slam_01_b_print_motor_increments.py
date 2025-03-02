# Print the increments of the left and right motor.
# Now using the LegoLogfile class.
# 01_b_print_motor_increments.py
# Motor tiklerinin bir önceki duruma göre farklarını verir

from lego_robot import LegoLogfile  # LegoLogfile sınıfını içe aktarır

if __name__ == '__main__':  # Programın ana kısmı, doğrudan çalıştırıldığında bu blok çalışır

    logfile = LegoLogfile()  # LegoLogfile sınıfından bir nesne oluşturur
    logfile.read("robot4_motors.txt")  # Belirtilen dosyayı okur ve motor tik verilerini yükler

    for i in range(20):  # İlk 20 motor tik verisini ekrana yazdırmak için bir döngü oluşturur
        print(logfile.motor_ticks[i])  # Motor tiklerinin ilk 20 satırını ekrana yazdırır
