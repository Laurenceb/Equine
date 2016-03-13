#include "wave.h"

/**
  * @brief  This function writes a WAV file header to a file using FAT-FS
  * @param  File pointer (file has already been opened), number of audio channels (0 to 255), sample rate as uint_16, bits in a sample (max 255)
  * @retval Result of file write operation
  */
FRESULT write_wave_header(FIL* file, uint8_t number_channels, uint16_t sample_rate, uint8_t bits_per_sample) {
	struct {
		uint8_t Chunkid[4];	//RIFF header
		uint8_t Chunksize[4];	//Use a stupidly large chunk size, technically this means a corrupt file, 
		uint8_t Format[4];	//but its simple and works (with octave)
		uint8_t Subchunk1id[4];
		uint8_t Subchunk1size[4];
		uint8_t Audioformat[2];	//Format 1 is PCM
		uint8_t Numchannels[2];	//Number of audio channels
		uint8_t Samplerate[4];	//Sample rate in samples per second
		uint8_t Byterate[4];	//Rate in bytes per second
		uint8_t Blockalign[2];	//Number of bytes for a block of channel samples
		uint8_t Bitspersample[2];//Bits in a single sample
		uint8_t Subchunk2id[4];
		uint8_t Subchunk2size[4];//Use largest chunk size
	} header= {{'R','I','F','F'},{0xFF,0xFF,0xFF,0x04},{'W','A','V','E'},{'f','m','t',' '},{0x10,0x00,0x00,0x00},{0x01,0x00},{},{},{},{},{},\
			{'d','a','t','a'},{0xDF,0xFF,0xFF,0x04}};
	header.Numchannels[0]=number_channels;
	*(uint16_t*)&(header.Samplerate)=sample_rate;//Works on little endian machines
	*(uint16_t*)&(header.Byterate)=(sample_rate*bits_per_sample*number_channels)/8;
	header.Blockalign[0]=(bits_per_sample*number_channels)/8;
	header.Bitspersample[0]=bits_per_sample;
	uint32_t bytes;
	return f_write(file, &header, sizeof(header), &bytes);//Write the header to the file
}

/**
  * @brief  This function write a WAV sample block to a file using FAT-FS
  * @param  File pointer (file has already been opened), number of audio channels (0 to 255), bits in a sample (max 255), wave_stuffer pointer
  *		holding bit stuffing context for the file, pointer to uint16_t variables holding the data samples (not all bits are used)
  * @retval Result of file write operation
  */
FRESULT write_wave_samples(FIL* file, uint8_t number_channels, uint8_t bits_per_sample, wave_stuffer* this_stuffer, void* data, uint8_t alignment) {
	uint8_t write_buffer[((bits_per_sample*number_channels)>>3)+1];//Sized to the amount of data (C99)
	memset(write_buffer,0,((bits_per_sample*number_channels)>>3)+1);
	write_buffer[0]=this_stuffer->lastbyte;	//Recover any data that was overhanding from the last write
	uint8_t offset_in_bits=this_stuffer->index;//This variable holds our index into the write buffer in units of bits
	for(uint8_t n=0;n<number_channels;n++) {
		uint32_t dat=*(uint32_t*)&(((uint8_t*)data)[n*alignment]);//read 32 bits from the source, jump by alignement bytes between samples	
		dat&=((uint32_t)(0x00000001<<bits_per_sample)-1);//mask off the upper bits
		*(uint32_t*)&write_buffer[offset_in_bits/8]|=dat<<(offset_in_bits%8);
		offset_in_bits+=bits_per_sample;
	}
	this_stuffer->index=offset_in_bits%8;	//The new bit overhang
	this_stuffer->lastbyte=write_buffer[offset_in_bits/8];//The byte containing the overhanging data
	uint32_t bytes;
	return f_write(file, write_buffer, offset_in_bits/8, &bytes);
}

/**
  * @brief  This function terminates a WAV file using FAT-FS
  * @param  File pointer (file has already been opened)
  * @retval Result of file operations
  */		
FRESULT wave_terminate(FIL* file) {
	FRESULT res;
	uint32_t b;
	res=f_sync(file);			//Flush buffers
	res|=f_truncate(file);			//Truncate the lenght - fix pre allocation
	DWORD size=f_size(file);
	DWORD subchunk=size;
	if (size & (DWORD)0x01)	 {		//Odd number of bytes
		uint8_t a=0;
		res|=f_write(file,&a,1,&b);	//Pad end of file with 0x00
		size++;				//New file size
	}
	res|=f_lseek(file,4);			//Seek to the chunk size
	DWORD chunk=(size-8);
	res|=f_write(file,&chunk,4,&b);
	res|=f_lseek(file,40);			//Seek to subchunk2 size
	chunk=subchunk-44;			//Size of the samples alone
	res|=f_write(file,&chunk,4,&b);
	f_sync(file);
	f_close(file);				//Close any opened file
}

