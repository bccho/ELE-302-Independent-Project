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
    string input;
    while (getline(cin, input)) {
        // Write input to Tx
        serialPuts(fd, string.c_str());

        // Receive from Rx
        char recByte = serialGetchar(fd);
    }

    return 0;
}

