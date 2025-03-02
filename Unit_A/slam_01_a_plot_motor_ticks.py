# Plot the ticks from the left and right motor.
# 01_a_plot_motor_ticks.py

from pylab import *

if __name__ == '__main__':  # Programın ana kısmı, doğrudan çalıştırıldığında bu blok çalışır
    # Listeden motor tik sayılarını alır. Motor tik sayıları başlangıçta 0'dan farklı olabilir. Aradaki fark önemlidir.

    f = open("robot4_motors.txt")  # Belirtilen dosyayı okuma modunda açar
    left_list = []  # Sol motor tik değerlerini saklamak için boş bir liste oluşturur
    right_list = []  # Sağ motor tik değerlerini saklamak için boş bir liste oluşturur
    
    for l in f:  # Dosyadaki her satırı döngüye alır
        sp = l.split()  # Satırı boşluklara göre ayırarak bir liste oluşturur

        left_list.append(int(sp[2]))  # Listenin 3. elemanını (2. indeks) tamsayıya çevirip sol motor listesine ekler
        right_list.append(int(sp[6]))  # Listenin 7. elemanını (6. indeks) tamsayıya çevirip sağ motor listesine ekler
        
    plot(left_list)  # Sol motor tik verilerini grafik olarak çizer
    plot(right_list)  # Sağ motor tik verilerini grafik olarak çizer
    show()  # Grafiği ekranda gösterir

