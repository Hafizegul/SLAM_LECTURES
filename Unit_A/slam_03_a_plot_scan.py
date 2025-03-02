# Plot a scan of the robot using matplotlib.
# 03_a_plot_scan


from pylab import *  # pylab kütüphanesini içe aktarır (grafik çizimi için kullanılır)
from lego_robot import *  # lego_robot modülünü içe aktarır

# Read the logfile which contains all scans.
logfile = LegoLogfile()  # LegoLogfile sınıfından bir nesne oluşturur
logfile.read("robot4_scan.txt")  # Tarama verilerini içeren dosyayı okur

# Plot one scan.
plot(logfile.scan_data[8])  # 8. tarama verisini grafik olarak çizer
show()  # Grafiği ekranda gösterir
