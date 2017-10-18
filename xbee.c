//
// Created by filip on 4/27/17.
//
#include "lib.h"

int set_interface_attribs(int fd, int speed, int parity) {

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        perror("error from tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, npeni echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN] = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // hut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("error from tcsetattr");
        return -1;
    }
    return 0;
}

void set_blocking(int fd, int should_block) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("error from tggetattr");
        return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        perror("error setting term attributes");
}

int myWrite(int fd, msg *m) {
    int count = 0;
    for (int i = 0; i < sizeof(msg); i++) {

        int n = write(fd, (char *)m + i, 1);

        if (n < 0) {
            return -1;
        }

        count++;
    }
    return count;
}

int myRead(int fd, msg *m) {
    int count = 0;
    for (int i = 0; i < sizeof(msg); i++) {

        int n = read(fd, (char*)m + i, 1);

        if (n < 0) {
            return -1;
        }

        count++;
    }
    return count;
}


int main(int argc, char **argv) {

    char *portname = "/dev/tty";
    char buffer[BUFLEN];
    memset(buffer, 0, BUFLEN);
    sprintf(buffer, "%s%s", portname, argv[1]);

    int fd = open(buffer, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("error opening");
        return 0;
    }

    fd_set read_fds;
    fd_set tmp_fds;
    int fdmax;

    FD_ZERO(&read_fds);
    FD_ZERO(&tmp_fds);

    FD_SET(0, &read_fds);
    FD_SET(fd, &read_fds);
    fdmax = fd;

    set_interface_attribs(fd, B115200, 0);
    set_blocking(fd, 0);

    msg recv;
    msg send;

    unsigned char seq = 0;
    unsigned char expected_seq;

    while (true) {
        tmp_fds = read_fds;

        if (select(fdmax + 1, &tmp_fds, NULL, NULL, NULL) == -1) {
            perror("ERROR in select\n");
            exit(1);
        }

        for (int i = 0; i <= fdmax; i++) {
            if (FD_ISSET(i, &tmp_fds)) {

                /*
                 * Daca i == 0 atunci se citeste de la stdin.
                 */
                if (i == 0) {

                    memset(&send, 0, sizeof(msg));
                    fgets(send.buffer, BUFLEN, stdin);
                    send.buffer[strlen(send.buffer) - 1] = '\0';

                    /*
                     * Transorm primele 4 litere (lower), deoarece vreau sa
                     * functioneze comanda 'quit' sa nu fie 'case sensitive'.
                     */
                    if (tolower(send.buffer[0]) == 'q'
                        && tolower(send.buffer[1]) == 'u'
                        && tolower(send.buffer[2]) == 'i'
                        && tolower(send.buffer[3]) == 't') {

                        return 0;
                    }

                    if (tolower(send.buffer[0]) == 't'
                        && tolower(send.buffer[1]) == 'e'
                        && tolower(send.buffer[2]) == 's'
                        && tolower(send.buffer[3]) == 't') {

                        send.type = TEST;

                        if (myWrite(fd, &send) < 0) {
                            printf("Eroare la trimitere. Se inchide conexiunea.\n");
                            return 0;
                        }

                        /*
                         * TODO VLAD
                         *
                         * Aici primesti mesaje generate random si numeri
                         * cate au CRC-ul bun. In functie de cate CRC bune ai
                         * afisezi un mesaj care iti zice cam cat de buna
                         * e conexiunea.
                         */

                        continue;
                    }

                    send.type = DATA;

                    /*
                     * TODO NICA & NICA
                     *
                     * criptare inainte de clacul CRC
                     */

                    send.CRC = crc16_ccitt(&send, BUFLEN + sizeof(char));

                    /*
                     * Pachetul cu mesajul citit de la tastatura este format, si
                     * va fi trimis pana voi primi ACK de la celalat capat.
                     */
                    do {

                        if (myWrite(fd, &send) < 0) {
                            printf("Eroare la trimitere. Se inchide conexiunea.\n");
                            return 0;
                        }

                        memset(&recv, 0, sizeof(msg));
                        sleep(1);

                        if (myRead(fd, &recv) < 0) {
                            printf("Eroare la citire. Se inchide conexiunea.\n");
                            return 0;
                        }


                        unsigned short CRC = crc16_ccitt(&recv, BUFLEN + sizeof(char));

                        if (CRC == recv.CRC
                            && recv.type == ACK
                            && strncmp(recv.buffer, "ACK", 3) == 0) {
                            break;
                        }

                    } while (true);
                }

                /*
                 * Daca i == fd, atunci primesc un mesaj.
                 */
                if (i == fd) {

                    boolean succes = false;

                    do {

                        memset(&recv, 0, sizeof(msg));

                        if (myRead(fd, &recv) < 0) {
                            printf("Eroare la citire. Se inchide conexiunea.\n");
                            return 0;
                        }

                        unsigned short CRC = crc16_ccitt(&recv, BUFLEN + sizeof(char));
                        memset(&send, 0, sizeof(msg));

                        if(CRC == recv.CRC && recv.type != DATA){
                            break;
                        }

                        if (CRC == recv.CRC) {

                            send.type = ACK;
                            sprintf(send.buffer, "%s", "ACK");
                            succes = true;

                        } else {

                            send.type = NAK;
                            sprintf(send.buffer, "%s", "NAK");

                        }

                        send.CRC = crc16_ccitt(&send, BUFLEN + sizeof(char));

                        if (myWrite(fd, &send) < 0) {
                            printf("Eroare la trimitere. Se inchide conexiunea.\n");
                            return 0;
                        }

                        if(succes) {
                            break;
                        }

                    } while (!succes);

                    if (recv.type == TEST) {

                        /*
                         * TODO LUCA
                         *
                         * Aici trimiti mai multe mesaje pe care le generezi
                         * random.
                         */

                    }

                    if (recv.type == DATA) {
                        /*
                         * TODO NICA & NICA
                         * decriptare
                         */
                        printf("Am primit mesajul: %s\n", recv.buffer);
                    }
                }
            }
        }
    }
}