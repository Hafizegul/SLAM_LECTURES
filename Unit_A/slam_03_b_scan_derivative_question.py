# Compute the derivative of a scan.
# 03_b_scan_derivative

from pylab import *  # pylab kütüphanesini içe aktar, genellikle matplotlib ve numpy işlevlerini içerir.
from lego_robot import *  # lego_robot modülünü içe aktar, robotla ilgili işlevler sağlar.

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):  # Derivatif hesaplayan fonksiyon, 'scan' (tarama verisi) ve 'min_dist' (geçerli mesafe) parametrelerini alır.
    jumps = [ 0 ]  # Derivatiflerin saklanacağı listeyi başlat, ilk değeri 0 olarak belirle.
    for i in range(1, len(scan) - 1):  # Tarama verilerinin başından sonuna kadar, ilk ve son eleman hariç her bir elemanı kontrol et.
        # --->>> Insert your code here.
        # Compute derivative using formula "(f(i+1) - f(i-1)) / 2".
        # Do not use erroneous scan values, which are below min_dist.
        # jumps.append(i%20 * 10) # Replace this line, append derivative instead.

        l = scan[i-1]  # 'i' konumundan bir önceki değeri al (sol komşu).
        r = scan[i+1]  # 'i' konumundan bir sonraki değeri al (sağ komşu).
        if l > min_dist and r > min_dist:  # Sol ve sağ komşu değerler geçerli mesafeden büyükse, geçerli bir ölçüm yapılıyor demektir.
            derivative = (r - l) / 2.0  # Derivatif hesaplamak için (sağ - sol) / 2 formülünü kullan.
            jumps.append(derivative)  # Hesaplanan derivatifi 'jumps' listesine ekle.
        else:
            jumps.append(0)  # Eğer bir komşu geçersizse, derivatifi 0 olarak ayarla.

    jumps.append(0)  # Son eleman için derivatifi 0 olarak ayarla.
    return jumps  # Hesaplanan derivatifleri içeren listeyi döndür.

if __name__ == '__main__':  # Eğer bu kod ana programda çalışıyorsa (modül olarak değilse).
    
    minimum_valid_distance = 20.0  # Geçerli mesafe olarak kabul edilecek minimum değeri belirle.

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()  # LegoLogfile sınıfından bir nesne oluştur.
    logfile.read("robot4_scan.txt")  # Tarama verilerini içeren log dosyasını oku.

    # Pick one scan.
    scan_no = 8  # Kullanılacak tarama verisinin numarasını belirle (bu durumda 8. tarama).
    scan = logfile.scan_data[scan_no]  # Seçilen taramayı log dosyasından al.

    # Compute derivative, (-1, 0, 1) mask.
    der = compute_derivative(scan, minimum_valid_distance)  # Seçilen tarama verisi için derivatif hesapla.

    # Plot scan and derivative.
    title("Plot of scan %d" % scan_no)  # Grafik başlığını seçilen tarama numarasına göre ayarla.
    plot(scan)  # Tarama verisini çiz.
    plot(der)  # Derivatif verisini çiz.
    show()  # Grafiği ekranda göster.
