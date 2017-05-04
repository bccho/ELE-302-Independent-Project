//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <Pixy.h>

Pixy pixy0(0x54);
Pixy pixy1(0x55);

int main()
{
    int j;
    uint16_t blocks;

    while (1) {
        // Pixy 0
        blocks = pixy0.getBlocks();

        if (blocks) {
            std::cout << "Pixy 0 detected " << blocks << std::endl;

            std::cout << "  largest block: ";
            pixy0.blocks[0].print();
            // for (j = 0; j < blocks; j++) {
                // std::cout << "  block " << j << " ";
                // pixy0.blocks[j].print();
            // }
        }

        // Pixy 1
        blocks = pixy1.getBlocks();

        if (blocks) {
            std::cout << "Pixy 1 detected " << blocks << std::endl;

            std::cout << "  largest block: ";
            pixy1.blocks[0].print();
            // for (j = 0; j < blocks; j++) {
                // std::cout << "  block " << j << " ";
                // pixy1.blocks[j].print();
            // }
        }
    }

    return 0;
}

