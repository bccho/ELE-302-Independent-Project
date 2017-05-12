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
    while (true) {
        int c;
        while ((c = serialGetchar(fd)) >= 0) {
            cout << (char) c;
        }
        cout << endl;
    }

    return 0;
}

