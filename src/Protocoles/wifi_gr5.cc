#include "../../include/Protocoles/wifi_gr5.h"


void handle_client(CtrlStruct *cvs, int new_socket) {
    char buffer[BUFFER_SIZE] = {0};
    int valread;

    while(1) {
        // Lecture des données
    valread = read(new_socket, buffer, BUFFER_SIZE);
    if (valread == 0) {
        printf("La connexion a été fermée par le client.\n");
        break;
    } else if (valread < 0) {
        perror("read");
        exit(EXIT_FAILURE);
    }

    // Extraction des valeurs de la chaîne reçue
    int values[6];
    char *token = strtok(buffer, " "); // Séparation par espace
    int i = 0;
    while (token != NULL && i < 6) {
        values[i++] = atof(token); // Conversion en entier
        token = strtok(NULL, " ");
    }
    // Affichage des valeurs reçues
    printf("Valeurs reçues : ");
    for (int j = 0; j < 6; j++) {
        printf("%d ", values[j]);
    }
    printf("\n");

    // printf("Valeur reçue : %s\n", buffer);
    memset(buffer, 0, BUFFER_SIZE);

    cvs->aruco->xR = values[0];
    cvs->aruco->yR = values[1];
    cvs->aruco->thetaR = values[2];

    cvs->aruco->xO = values[3];
    cvs->aruco->yO = values[4];
    cvs->aruco->thetaO = values[5];
    printf("Valeur reçue : %f, %f, %f\n", cvs->aruco->xR, cvs->aruco->yR, cvs->aruco->thetaR);

    }
}

void wifi(CtrlStruct *cvs) {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Création du socket serveur
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    
    // Configuration du socket
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
    
    // Attachement du socket au port et adresse
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    
    // Attente de connexion
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    printf("En attente de connexion...\n");
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    printf("Connexion établie\n");
    
    // Gestion du client
    handle_client(cvs, new_socket);

    close(new_socket);
    close(server_fd);
}
