#include <iostream>
#include <wiringSerial.h>

using namespace std;

int main() {
    /* Open serial connection */
    int baud = 9600;
    int fd = serialOpen("/dev/ttyAMA0", baud);
    if (fd >= 0) { cout << "Serial opened at " << baud << " baud" << endl; }
    else         { cout << "Failed to open serial!" << endl; return -1; }

    /* Receive input, display output */
    string data;
    string input;
    while (getline(cin, input)) {
        serialPuts(fd, (input + "\n").c_str());

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

    }

    return 0;
}

