# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, use this transform to correct the pose.
# 04_d_apply_transform

from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders
from math import sqrt, atan2
import numpy as np
import math

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
# This is the function developed in slam_04_b_find_cylinder_pairs.
# Taranan silindirler ile referans silindirler arasındaki en yakın eşleşen çiftleri bulur.
# İki nokta arasındaki Öklid mesafesini hesaplayan fonksiyon.
def compute_distance(i, j):
    x = i[0] - j[0]
    y = i[1] - j[1]
    return np.sqrt(x*x + y*y)  # Mesafe formülü: √((x2-x1)² + (y2-y1)²)

def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []  # Eşleşen çiftleri tutacak liste.

    for i, cyl in enumerate(cylinders):
        for j, rcyl in enumerate(reference_cylinders):
            if compute_distance(cyl, rcyl) < max_radius:  # Mesafe maksimum mesafeden küçükse eşleştir.
                cylinder_pairs.append((i, j))

    return cylinder_pairs  # Eşleşen silindir indeks çiftlerini döndür.


# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (sx / len(point_list), sy / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
# İki nokta kümesi arasındaki uygun dönüşümü hesaplar.
def estimate_transform(left_list, right_list, fix_scale=False):
    lc = compute_center(left_list)  # Dünya koordinatındaki silindirlerin merkezi.
    rc = compute_center(right_list)  # Referans silindirlerin merkezi.

    # Noktaları, kütle merkezi etrafında döndürme ve kaydırma için yeniden konumlandır.
    l_prime = [(l[0]-lc[0], l[1]-lc[1]) for l in left_list]
    r_prime = [(r[0]-rc[0], r[1]-rc[1]) for r in right_list]

    cs, ss, rr, ll = 0, 0, 0, 0  # Toplamalar için değişkenler.

    for i in range(len(left_list)):
        l, r = l_prime[i], r_prime[i]

        cs += (r[0] * l[0]) + (r[1] * l[1])
        ss += (-r[0] * l[1]) + (r[1] * l[0])
        rr += (r[0] * r[0]) + (r[1] * r[1])
        ll += (l[0] * l[0]) + (l[1] * l[1])

    if ll < 0.00001:  # Bölme hatasını önlemek için kontrol.
        return None

    la = 1 if fix_scale else np.sqrt(rr / ll)  # Ölçek sabitlenmişse 1, yoksa hesaplanır.

    cs_sum = np.sqrt(cs*cs + ss*ss)
    c = cs / cs_sum  # Kosinüs açısı.
    s = ss / cs_sum  # Sinüs açısı.

    # Dönüşüm matrisindeki öteleme bileşenlerini hesapla.
    tx = rc[0] - la * (c * lc[0] - s * lc[1])
    ty = rc[1] - la * (s * lc[0] + c * lc[1])

    return la, c, s, tx, ty  # Ölçek, açı (cos,sin) ve öteleme bileşenleri.

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

# Correct the pose = (x, y, heading) of the robot using the given
# similarity transform. Note this changes the position as well as
# the heading.
def correct_pose(pose, trafo):
    x, y, theta = pose
    scale, cos_alpha, sin_alpha, dx, dy = trafo

    # Pozisyon güncellenir
    x_new = cos_alpha * x - sin_alpha * y + dx
    y_new = sin_alpha * x + cos_alpha * y + dy

    # Açıyı güncelle (rotasyon açısını ekle)
    theta_new = theta + math.atan2(sin_alpha, cos_alpha)

    return (x_new, y_new, theta_new)


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 400.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Also read the reference cylinders (this is our map).
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

   
    with open("apply_transform.txt", "w") as out_file:
        for i in range(len(logfile.scan_data)):
            # Compute the new pose.
            pose = filter_step(pose, logfile.motor_ticks[i],
                               ticks_to_mm, robot_width,
                               scanner_displacement)

            # Extract cylinders, also convert them to world coordinates.
            cartesian_cylinders = compute_scanner_cylinders(
                logfile.scan_data[i],
                depth_jump, minimum_valid_distance, cylinder_offset)

            world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                               for c in cartesian_cylinders]

            # For every cylinder, find the closest reference cylinder.
            cylinder_pairs = find_cylinder_pairs(
                world_cylinders, reference_cylinders, max_cylinder_distance)

            # Estimate a transformation using the cylinder pairs.
            trafo = estimate_transform(
                [world_cylinders[pair[0]] for pair in cylinder_pairs],
                [reference_cylinders[pair[1]] for pair in cylinder_pairs],
                fix_scale=True)

            # Transform the cylinders using the estimated transform.
            transformed_world_cylinders = []
            if trafo:
                transformed_world_cylinders = [
                    apply_transform(trafo, c) for c in
                    [world_cylinders[pair[0]] for pair in cylinder_pairs]]

            # Also apply the trafo to correct the position and heading.
            if trafo:
                pose = correct_pose(pose, trafo)

            # Write to file.
            out_file.write("F %f %f %f\n" % pose)

            # The detected cylinders in the scanner's coordinate system.
            write_cylinders(out_file, "D C", cartesian_cylinders)

            # The detected cylinders, transformed using the estimated trafo.
            write_cylinders(out_file, "W C", transformed_world_cylinders)

