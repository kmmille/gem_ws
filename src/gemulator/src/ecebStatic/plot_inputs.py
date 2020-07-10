import csv
import matplotlib.pyplot as plt

times = []
vel = []
ang = []
with open('inputs.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        if times == []:
            t0 = float(row[0])
        times.append(float(row[0]) - t0)
        vel.append(float(row[1]))
        ang.append(float(row[2]))

plt.subplot(1, 2, 1)
plt.scatter(times, vel, color = 'r')
plt.plot(times, vel, color = 'r')
plt.title('Velocity Input')
plt.xlabel('time')
plt.ylabel('velocity')
plt.subplot(1, 2, 2)
plt.scatter(times, ang, color = 'r')
plt.plot(times, ang, color = 'r')
plt.title('Angular Velocity Input')
plt.xlabel('time')
plt.ylabel('angular velocity')
plt.show()
