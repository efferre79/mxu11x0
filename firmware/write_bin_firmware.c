#include <stdio.h>

#include "mxu1110_fw.h"
#include "mxu1130_fw.h"
#include "mxu1131_fw.h"
#include "mxu1150_fw.h"
#include "mxu1151_fw.h"

int tr(char* out, unsigned char buffer[], int size) {
	FILE* fo = fopen(out, "wb");    
   
	int buffer_size = size * sizeof(char) + 1;
	
	fwrite(buffer, buffer_size, 1, fo);
	
	fclose(fo);	
}

int main() {
    
	tr("./moxa-1110.fw", mxu1110FWImage, sizeof(mxu1110FWImage));
	tr("./moxa-1130.fw", mxu1130FWImage, sizeof(mxu1130FWImage));
	tr("./moxa-1131.fw", mxu1131FWImage, sizeof(mxu1131FWImage));
	tr("./moxa-1150.fw", mxu1150FWImage, sizeof(mxu1150FWImage));
	tr("./moxa-1151.fw", mxu1151FWImage, sizeof(mxu1151FWImage));
	
	return 0;
}
