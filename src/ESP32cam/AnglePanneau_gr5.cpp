#include <iostream>
#include <fstream>
#include <cstdlib>

#include "../../include/ESP32cam/AnglePanneau_gr5.h"

int main() {
    // Exécute le script Python en utilisant la commande système
    system("python3 /home/raspberry/Documents/RobotCtrl/Mecatronics/src/ESP32cam/AngleAruco.py");

    std::ifstream file("angle.txt");
    if (!file.is_open()) {
        std::cout << "Erreur lors de l'ouverture du fichier" << std::endl;
        return 1;
    }

    // Lire les données à partir du fichier
    AnglePanel data;
    file >> data.angle;

    // Fermer le fichier
    file.close();

    // Afficher les données lues
    std::cout << "Angle : " << std::fixed << data.angle << std::endl;

    return 0;
}
