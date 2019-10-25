#include <stdio.h>
#include <stdlib.h>

int to_little(int bits32){
	unsigned char bytes[4];
	
	bytes[0] = (unsigned char)((bits32 >> 0) &0xff);
	bytes[1] = (unsigned char)((bits32 >> 8) &0xff);
	bytes[2] = (unsigned char)((bits32 >> 16) &0xff);
	bytes[3] = (unsigned char)((bits32 >> 24) &0xff);
	return	((int)bytes[0]<<0)|
			((int)bytes[1]<<8)|
			((int)bytes[2]<<16)|
			((int)bytes[3]<<24);
}

int main(int argc, char *argv[]){
	FILE *binary;
	FILE *hexa;
	int instruction_count = 0;
	int ch;

	if(argc != 3){
		printf("Usage: %s (input binary file) (output hex file name)\n",argv[0]);
		return 1;
	}

	binary = fopen(argv[1], "rb");
	hexa = fopen(argv[2], "w");


	// while True:
	while(1){
		int tmp[4]={0};
	//	instruction = binary.read(4)
	//	if instruction == '' : break
		if ((tmp[0] = fgetc(binary))==EOF)
			break;
		tmp[1] = fgetc(binary);
		tmp[2] = fgetc(binary);
		tmp[3] = fgetc(binary);

		fprintf(hexa,"%08X\n", to_little(((int)tmp[0]<<0)|((int)tmp[1]<<8)|((int)tmp[2]<<16)|((int)tmp[3]<<24)));
	//	instruction_count += 1
		instruction_count++;
	//	bytes = map(ord, instruction)	
	//	bytes.reverse()
		// printf("int %d, %x\n", instruction_count,instruction);
	//	# for byte in bytes: hexa.write('%02x' % byte)
        
	//	hex = lambda x: ('%02x' % x)
	//	bytes = map(hex, bytes)
	//	hexa.write('_'.join(bytes))
        
	//	hexa.write('\n')

	}


	fclose(binary);
	fclose(hexa);	

	printf("Converted %d instructions (binary -> hex string)\n",instruction_count);

	return 0;
}

/*
import sys

if len(sys.argv) < 3:
    print >> sys.stderr, 'Usage: python %s (input binary file) (output hex file name)' % (sys.argv[0])
    sys.exit(0)

try:
    binary = open(sys.argv[1], 'rb')
    hexa = open(sys.argv[2], 'w')

    instruction_count = 0

    while True:
        instruction = binary.read(4)
        if instruction == '' : break

        instruction_count += 1

        bytes = map(ord, instruction)
        bytes.reverse()

        # for byte in bytes: hexa.write('%02x' % byte)
        
        hex = lambda x: ('%02x' % x)
        bytes = map(hex, bytes)
        hexa.write('_'.join(bytes))
        
        hexa.write('\n')

    hexa.close()
    binary.close()

    print 'Converted %d instructions (binary -> hex string)' % instruction_count

except IOError:
    print >> sys.stderr, 'Fail to open the file!'
    sys.exit(0)


*/