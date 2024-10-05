#include <stdio.h>
#include <stdint.h>
#include <signal.h>
/* unix only */
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>

#define MEMORY_MAX (1 << 16)
uint16_t memory[MEMORY_MAX];

enum
{
    MR_KBSR = 0xFE00,
    MR_KBDR = 0xFE02

};

enum
{
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC,
    R_COND,
    R_COUNT
};
uint16_t reg[R_COUNT];

enum
{
    OP_BR = 0,  /* branch */
    OP_ADD,     /* add */
    OP_LD,      /* load */
    OP_ST,      /* store */
    OP_JSR,     /* jump register */
    OP_AND,     /* bitwise and */
    OP_LDR,     /* load register */
    OP_STR,     /* store register */
    OP_RTI,     /* unused */
    OP_NOT,     /* bitwise not */
    OP_LDI,     /* load indirect */
    OP_STI,     /*store indirect */
    OP_JMP,     /* jump */
    OP_RES,     /* reserved (unused)*/
    OP_LEA,     /* load effective address*/
    OP_TRAP     /* execute trap */
};

enum
{
    FL_POS = 1 << 0, /* P */
    FL_ZRO = 1 << 1, /* Z */
    FL_NEG = 1 << 2, /* N */
};

//TRAP codes
enum
{
    TRAP_GETC = 0x20, /* get character from keyboard, not echoed onto the terminal*/
    TRAP_OUT, /* output a character*/
    TRAP_PUTS, /* output a word string*/
    TRAP_IN, /* get character from keyboard, echoed onto the terminal*/
    TRAP_PUTSP, /* output a byte string*/
    TRAP_HALT /* halt the program*/
};

//Main Loop

//Prototyping
uint16_t sign_extend(uint16_t, int);
void update_flags(uint16_t);

struct termios original_tio;

void disable_input_buffering()
{
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

uint16_t check_key()
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}
void handle_interrupt(int signal)
{
    restore_input_buffering();
    printf("\n");
    exit(-2);
}

uint16_t sign_extend(uint16_t x, int bit_count)
{
    if ((x >> (bit_count - 1)) & 1) {
        x |= (0xFFFF << bit_count);
    }
    return x;
}

uint16_t swap16(uint16_t x){
    return ((x << 8) | (x >> 8));
}

void update_flags(uint16_t r)
{
    if (reg[r] == 0)
    {
        reg[R_COND] = FL_ZRO;
    }
    else if (reg[r] << 15) /* negatives have a 1 at bit 15 (16th position from right)*/
    {
        reg[R_COND] = FL_NEG;
    }
    else
    {
        reg[R_COND] = FL_POS;
    }
}


void read_image_file(FILE* file){
    uint16_t origin;
    fread(&origin, sizeof(origin), 1, file);

    //swap16 swaps from little endian to big endian, and vice versa
    origin = swap16(origin);

    /* this defines where we will start putting our instructions in memory */
    uint16_t max_read = MEMORY_MAX - origin;
    uint16_t* p = memory + origin;
    size_t read = fread(p, sizeof(uint16_t), max_read, file);

    while (read-- > 0){
        *p = swap16(*p);
        p++;
    }

}

int read_image(const char* image_path) {
    FILE* file = fopen(image_path, "rb");
    if (!file) {return 0;};
    read_image_file(file);
    fclose(file);
    return 1;

}
//Mem read and write go here
uint16_t mem_write(u_int16_t address, uint16_t val){
    memory[address] = val;
}

uint16_t mem_read(uint16_t address){

    //check if the address is our special regs
    if (address == MR_KBSR)
    {
        if (check_key())
        {
            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBSR] = getchar();
        }
        else
        {
            memory[MR_KBSR] = 0;
        }
    }
    return memory[address];
}


int main(int argc, const char* argv[])
{
    /* LOAD ARGUMENTS uses the command line input*/
    if (argc < 2)
    {
        /* showing usage string */
        printf("lc3 [image-file1] ...\n");
        exit(2);
    }

    for (int j = 1; j < argc; ++j)
    {
        if (!read_image(argv[j]))
        {
            printf("failed to load image: %s\n", argv[j]);
            exit(1);
        }
    }
    /* SETUP */
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();


    reg[R_COND] = FL_ZRO;

    enum { PC_START = 0x3000};
    reg[R_PC] = PC_START;

    int running = 1;
    while (running)
    {
        /* FETCH */
        uint16_t instr = mem_read(reg[R_PC]++);
        uint16_t op = instr >> 12;

        //OPCODE implements
        switch (op)
        {
            case OP_ADD:
                /*ADD goes here*/

                /* Destination Register*/
                uint16_t dr = (instr >> 9) & 0x7;
                /* Source Register 1*/
                uint16_t sr1 = (instr >> 6) & 0x7;
                /* Flag to check for negatives*/
                uint16_t imm_flag = (instr >> 5) & 0x1;
                
                if (imm_flag)
                {
                    uint16_t imm5 = sign_extend(instr & 0x1F, 5);
                    reg[dr] = reg[sr1] + imm5;
                }
                else
                {
                    uint16_t sr2 = instr & 0x7;
                    reg[dr] = reg[sr1] + reg[sr2];
                }
                update_flags(dr);
                

                break;
            case OP_AND:
                /* AND goes here*/
                dr = (instr >> 9) & 0x7;
                sr1 = (instr >> 6) & 0x7;
                imm_flag = (instr >> 5) & 1;

                if (imm_flag)
                {
                    uint16_t imm5 = (instr & 0x1F, 5);
                    reg[dr] = reg[sr1] & imm5;
                }
                else
                {
                    uint16_t sr2 = instr & 0x7;
                    reg[dr] = reg[sr1] & reg[sr2];
                }
                update_flags(dr);
                break;
            case OP_NOT:
                /* NOT goes here */

                dr = (instr >> 9) & 0x7;
                uint16_t sr = (instr >> 6) & 0x7;

                reg[dr] = !(reg[sr]);
                update_flags(dr);
                break;
            case OP_BR:
                /* BR goes here*/
                uint16_t cond_flag = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
                if (cond_flag & reg[R_COND]) //nonzeros are true too...
                {
                    reg[R_PC] += pc_offset;
                }
                break;
            case OP_JMP:
                /* JMP goes here*/

                uint16_t r0 = (instr >> 6) & 0x7;
                reg[R_PC] = reg[r0];
                break;
            case OP_JSR:
                /* JSR goes here*/
                reg[R_R7] = reg[R_PC];
                uint16_t offset_flag = (instr >> 11) & 0x1;

                if (offset_flag)
                {
                    pc_offset = sign_extend(instr & 0x7FF, 11);
                    reg[R_PC] += pc_offset;
                }
                else
                {
                    uint16_t baseR = (instr >> 6) & 0x7;
                    reg[R_PC] = reg[baseR];
                }
                break;
            case OP_LD:
                /* LD goes here */
                dr = (instr >> 9) & 0x7;
                pc_offset = sign_extend(instr & 0x1FF, 9);
                reg[dr] = mem_read(reg[R_PC] + pc_offset);
                update_flags(dr);
                break;
            case OP_LDI:
                /* LDI goes here */
                
                dr = (instr >> 9) & 0x7;
                pc_offset = sign_extend(instr & 0x1FF, 9);
                reg[dr] = mem_read(mem_read(reg[R_PC] + pc_offset));
                update_flags(dr);
                break;
            case OP_LDR:
                /* LDR goes here */
                dr = (instr >> 9) & 0x7;
                uint16_t baseR = (instr >> 6) & 0x7;
                uint16_t offset = sign_extend(instr & 0x3F, 6);

                reg[dr] = mem_read(reg[baseR] + offset);
                update_flags(dr); 

                break;
            case OP_LEA:
                /* LEA goes here */
                dr = (instr >> 9) & 0x7;
                pc_offset = sign_extend(instr & 0x1FF, 9);

                reg[dr] = reg[R_PC] + pc_offset;
                update_flags(dr);
                break;
            case OP_ST:
                /* ST goes here*/
                sr= (instr >> 9) & 0x7;
                pc_offset = sign_extend(instr & 0x1FF, 9);

                mem_write(reg[sr], reg[R_PC] + pc_offset);
                break;
            case OP_STI:
                /* STI goes here*/
                sr= (instr >> 9) & 0x7;
                pc_offset = sign_extend(instr & 0x1FF, 9);

                mem_write(mem_read(reg[R_PC] + pc_offset), reg[sr]);
                break;
            case OP_STR:
                /* STR goes here*/
                sr= (instr >> 9) & 0x7;
                baseR = (instr >> 6) & 0x7;
                offset = sign_extend(instr & 0x3F, 6);

                mem_write(reg[sr], reg[baseR] + offset);
                break;
            case OP_TRAP:
                /* TRAP goes here*/
                reg[R_R7] = reg[R_PC];
                switch (instr & 0xFF)
                {
                    case TRAP_GETC:
                        //does a single get character exist?
                        reg[R_R0] = (uint16_t)getc(stdin);
                        fflush(stdin);
                        update_flags(R_R0);
                    break;

                    case TRAP_OUT:
                        putc((char)reg[R_R0], stdout);
                        fflush(stdout);
                    break;

                    case TRAP_PUTS:
                        uint16_t* c = memory + reg[R_R0];
                        while (*c){
                            putc((char)*c, stdout);
                            ++c;
                        }
                        fflush(stdout);
                    break;

                    case TRAP_IN:
                        putc('>', stdout);
                        fflush(stdout);
                        c = getc(stdin);
                        putc(c, stdout);
                        reg[R_R0] = (uint16_t)c;
                        update_flags;
                    break;

                    case TRAP_PUTSP:
                        c = memory + reg[R_R0];
                        while (*c){

                            char char1 = (*c) & 0xFF;
                            putc(char1, stdout);
                            char char2 = (*c) & 0xFF00;
                            if (char2) putc(char2, stdout);
                            c++;
                        }
                        fflush(stdout);
                    break;
                    case TRAP_HALT:
                        printf("Halting execution.\n");
                        running = 0;
                    break;
                }
                break;
            case OP_RES:
            case OP_RTI:
            default:
                /* BAD OPCODE goes here*/
                abort();
                break;

        }
    }
    /* SHUTDOWN GOES HERE*/
    restore_input_buffering();

}

