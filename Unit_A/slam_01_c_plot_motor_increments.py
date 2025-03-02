# Plot the increments of the left and right motor.
# 01_c_plot_motor_increments.py


from pylab import *  # pylab kütüphanesini içe aktarır (grafik çizimi için kullanılır)
from lego_robot import LegoLogfile  # LegoLogfile sınıfını içe aktarır

if __name__ == '__main__':  # Programın ana kısmı, doğrudan çalıştırıldığında bu blok çalışır

    logfile = LegoLogfile()  # LegoLogfile sınıfından bir nesne oluşturur
    logfile.read("robot4_motors.txt")  # Belirtilen dosyayı okur ve motor tik verilerini yükler

    plot(logfile.motor_ticks)  # Motor tik verilerini grafik olarak çizer
    show()  # Grafiği ekranda gösterir
