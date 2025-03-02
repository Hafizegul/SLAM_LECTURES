# Implement the first move model for the Lego robot.
# 02_a_filter_motor


from math import sin, cos, pi  # Trigonometrik fonksiyonlar ve pi sayısını içe aktarır
from pylab import *  # pylab kütüphanesini içe aktarır (grafik çizimi için kullanılır)
from lego_robot import *  # lego_robot modülünü içe aktarır

def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):
    # Aracın yeni pozisyonunu hesaplayan fonksiyon

    if motor_ticks[0] == motor_ticks[1]:  # Eğer her iki motorun tik sayısı eşitse, araç düz hareket eder

        '''
            Eğer encoder değerleri eşit ise araç doğrultusu yönünde düz hareket edecektir. 
        --> Hareket uzunluğu yani l, encoder tick sayısı ve her tickte aracın kaç mm ilerlediği çarpılarak bulunur.
        --> Aracın yeni koordinatları, mevcut koordinatların üzerine, l uzunluğunun doğrultu açısıyla çarpılmasıyla elde edilen değerin eklenmesiyle bulunur. 
        '''

        x, y, theta = old_pose  # Mevcut konumu alır
        x = x + ((motor_ticks[0]*ticks_to_mm)*cos(theta))  # Yeni x koordinatını hesaplar
        y = y + ((motor_ticks[1]*ticks_to_mm)*sin(theta))  # Yeni y koordinatını hesaplar

        return (x, y, theta)  # Yeni pozisyonu döndürür

    else:

        '''
            Eğer encoder değerleri birbirine eşit değilse, araç encoder değeri az olan motorun bulunduğu tarafa doğru yönelerek hareket eder. Her bir palet
        başlangıç ve bitiş noktaları arasında birer yay çizer. Uzun yay "r", kısa yay "l" harfi ile gösterilir. (Uzunluk hesabı line-20'de belirtilmiştir.)

            Yayların iki uç noktasının kesişimi olan noktaya c noktası dersek. c noktasının koordinatları line-50/51 deki formül aracılığıyla bulunur. c noktasının
        x ve y koordinatları arasında kalan açıya alfa açısı denir. R c noktasıyla aracın başlangıç noktası arasındaki uzunluk yani küçük dairenin çapıdır.
        Büyük dairenin çapı ise aracın genişliği/2 + R'dir.  
        '''

        l = (motor_ticks[0]*ticks_to_mm)  # Sol tekerin ilerlediği mesafeyi hesaplar
        r = (motor_ticks[1]*ticks_to_mm)  # Sağ tekerin ilerlediği mesafeyi hesaplar

        alpha = (r-l)/robot_width  # Dönüş açısını hesaplar
        R = l/alpha  # Dönüş yarıçapını hesaplar

        x, y, theta = old_pose  # Mevcut konumu alır

        cx = x - ((R + (robot_width/2))*sin(theta))  # Dönüş merkezinin x koordinatını hesaplar
        cy = y - ((R + (robot_width/2))*-cos(theta))  # Dönüş merkezinin y koordinatını hesaplar

        theta = (theta + alpha) % (2*pi)  # Yeni açıyı hesaplar ve 2π ile mod alarak normalize eder

        x = cx + ((R + (robot_width/2))*sin(theta))  # Yeni x koordinatını hesaplar
        y = cy + ((R + (robot_width/2))*-cos(theta))  # Yeni y koordinatını hesaplar

        return (x, y, theta)  # Yeni pozisyonu döndürür

if __name__ == '__main__':
    ticks_to_mm = 0.349  # Motor tiklerinin mm'ye dönüşüm katsayısı
    robot_width = 150.0  # Robotun genişliği (mm cinsinden)

    logfile = LegoLogfile()  # LegoLogfile sınıfından bir nesne oluşturur
    logfile.read("robot4_motors.txt")  # Motor tik verilerini içeren dosyayı okur

    pose = (0.0, 0.0, 0.0)  # Başlangıç pozisyonu (x, y, theta)

    filtered = []  # Filtrelenmiş pozisyonları saklamak için liste oluşturur
    for ticks in logfile.motor_ticks:  # Motor tik verilerini sırayla işler
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width)  # Yeni pozisyonu hesaplar
        filtered.append(pose)  # Listeye ekler

    for pose in filtered:  # Filtrelenmiş pozisyonları ekrana yazdırır
        print(pose)  # Pozisyonu ekrana basar
        plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')  # Pozisyonları grafiğe çizer
    show()  # Grafiği ekranda gösterir
