// Online C compiler to run C program online
#include <stdio.h>
#include <stdint.h>


unsigned int bitExtracted(unsigned int number, int k, int p)
{
    return (((1 << k) - 1) & (number >> (p - 1)));
}

int reverse_bits_recursive(unsigned int num, unsigned int numBits)
{
    unsigned int reversedNum;;
    unsigned int mask = 0;

    mask = (0x1 << (numBits/2)) - 1;

    if (numBits == 1) return num;
    reversedNum = reverse_bits_recursive(num >> numBits/2, numBits/2) | reverse_bits_recursive((num & mask), numBits/2) << numBits/2;
    return reversedNum;
}

void writeByte(uint16_t byte, int bitsToExtract, int startBit)
{
	uint16_t reversedNumber, dummy, extractedNumber, extractedNumber1;
	
	reversedNumber = reverse_bits_recursive(byte, 16);
    extractedNumber = bitExtracted(reversedNumber, bitsToExtract, startBit);
	extractedNumber1 = bitExtracted(reversedNumber, bitsToExtract, startBit);
	
	printf("Original is %u\n", byte);
    printf("After reversed & extracted is %u", extractedNumber);
	
	printf("\n\nOriginal:\n");
	for (int i=0; i<16; i++)
	{
		dummy = byte&0x0001;
		printf("%u ", dummy);
		byte = byte >> 1;
	}
	
	printf("\nReversed:\n");
	for (int i=0; i<16; i++)
	{
		dummy = reversedNumber&0x0001;
		printf("%u ", dummy);
		reversedNumber = reversedNumber >> 1;
	}
	
	printf("\nAfter Reversed & Exctracted:\n");
	for (int i=0; i<16; i++)
	{
		dummy = extractedNumber&0x0001;
		printf("%u ", dummy);
		extractedNumber = extractedNumber >> 1;
	}
	
	printf("\nExtracted bits to send as Command:\n");
	for (int i=0; i<bitsToExtract; i++)
	{
		dummy = extractedNumber1&0x0001;
		printf("%u ", dummy);
		extractedNumber1 = extractedNumber1 >> 1;
	}
}


int main() {
    // Write C code here
    unsigned int value = 0x0830;
    writeByte(value, 12, 5);

    return 0;
}