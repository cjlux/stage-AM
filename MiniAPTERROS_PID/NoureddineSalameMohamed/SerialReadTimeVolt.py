# Programme à exécuter avec Python3

#
# Ce programme fait la lecture de la ligne série.
# Les données attendues sont des lignes de la forme :
# "temps_milliseconde \t tension"
# Quand on reçoit juste un "-1", c'est la marque du STOP.
#
from serial import Serial
import numpy as np

# Connexion au port série :
# Nom du port série  (le nom est dans la barre de titre du 'Moniteur') :
#   Sous Windows : "COM3" ou "COM4"....
#   Sous Linux   : "/dev/ttyACM0" (ou voir barre de titre fenêtre Moniteur)
#   Sous Mac OS X: voir barre de titre fenêtre Moniteur
#
# Paramètres : Nbre de bits de données -> 8
#              Nbre de bit STOP        -> 1
#              Parité                  -> Sans (None)
#              Vitesses                -> 9600, 14400, 19200, 28800, 38400,
#                                         57600, or 115200
#

serPort = Serial("/dev/ttyACM0", baudrate=19200, timeout=None)
print(serPort) # juste pour voir...

# liste de lecture des données:
lu = []

# Boucle de lecture des données sur le port série :
while True:
    data = serPort.readline()   # Lire une ligne de données
    data = data.strip()         # Enlever les caractères de fin de ligne
    if data == b'-1':           # "-1 : Bouton poussoir STOP appuyé
        serPort.close()         # Fermer le port série
        print("Fin")
        break                   # sortir de la boucle while
    if data is not "":          # on a lu quelquechose
        data = list(map(float,data.split())) # conversion en liste de flottants
        print(data)             # contrôle à l'affichage...
        if len(data) > 1:       # plusieurs données sont lues (dont le temps),
            lu.append(data)     # tout va bien, on ajoute les données à la liste 'lu'.

#
# à partir d'ici, on est sorti de la boucle de lecture des données            
#
lu = np.array(lu)               # convertir la liste en objet ndarray
X  = lu[:,0]                    # colonne 0 : vecteur temps 
Y  = lu[:,1]                    # colonne 1 : tension 

#
# Écriture des données dans un fichier
#
from time import strftime
uniqFileName = "TensionData_"+strftime("%Y-%m-%d_%H:%M:%S")+".txt"
fOut = open(uniqFileName, "w");
LF = "\r\n"                     # LineFeed : saut de ligne ASCII
fOut.write("#Temps [ms]\tTension [V]"+LF)
for x, y in zip(X,Y):
    fOut.write("{:f}\t{:f}{:s}".format(x, y, LF))
fOut.close()

#
# tracé de la courbe
#
import matplotlib.pyplot as plt
plt.figure()
plt.grid(True)
plt.xlabel("Temps [s]")
plt.ylabel("Tension [V]")
plt.ylim(-0.1,5.1)
plt.plot(X/1000, Y, "o-b")
plt.show()
