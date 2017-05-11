#include <iostream>
#include <wiringSerial.h>

using namespace std;

int main() {
    /* Open serial connection */
    int baud = 115200;
    int fd = serialOpen("/dev/ttyAMA0", baud);
    if (fd >= 0) { cout << "Serial opened at " << baud << " baud" << endl; }
    else         { cout << "Failed to open serial!" << endl; return -1; }

    /* Receive input, display output */
    string data;
    while (true) {
        while (serialDataAvail(fd)) {
            /* Receive */
            // get char
            char recByte = serialGetchar(fd);
            // if NL, CR, or EOT, indicate end of message, print special char, and
            // print message in received buffer
            if (recByte == 0xA || recByte == 0xD || recByte == 0x4) {
                cout << recByte;
                cout << data;
                data = "";
            } else if (recByte > 0) {
                // put byte in buffer and echo the byte
                data += recByte;
            // byte should never be zero, so print error char if it is
            } else {
                cout << "!";
            }
        }

        /* Listen to input and transmit */
        string input;
        if (cin >> input) {
            serialPuts(fd, (input + "\n").c_str());
        }
    }

    return 0;
}

