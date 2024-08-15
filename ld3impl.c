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

//Main Loop

//Prototyping
uint16_t sign_extend(uint16_t, int);
void update_flags(uint16_t);

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


    reg[R_COND] = FL_ZRO;

    enum { PC_START = 0x3000}
    reg[R_PC] = PC_START;

    int running = 1;
    while (running)
    {
        /* FETCH */
        uint16_t instr = mem_read(reg[R_PC]++);
        uint16_t op = instr >> 12;

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
                update_flags(reg[dr]);
                

                break;
            case OP_AND:
                /* AND goes here*/
                uint16_t dr = (instr >> 9) & 0x7;
                uint16_t sr1 = (instr >> 6) & 0x7;
                uint16_t imm_flag = (instr >> 5) & 1;

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
                update_flags(reg[dr]);
                break;
            case OP_NOT:
                /* NOT goes here */

                uint16_t dr = (instr >> 9) & 0x7;
                uint16_t sr = (instr >> 6) & 0x7;

                reg[dr] = !(reg[sr]);
                update_flags(reg[dr]);
                break;
            case OP_BR:
                /* BR goes here*/
                uint16_t cond_flag = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
                if (cond_flag & reg[R_COND])
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
                    uint16_t pc_offset = sign_extend(instr & 0x3FF, 10);
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
                uint16_t dr = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
                reg[dr] = mem_read(reg[R_PC] + pc_offset);
                update_flags(reg[dr]);
                break;
            case OP_LDI:
                /* LDI goes here */
                
                uint16_t dr = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
                reg[dr] = mem_read(mem_read(reg[R_PC] + pc_offset));
                update_flags(reg[dr]);
                break;
            case OP_LDR:
                /* LDR goes here */
                uint16_t dr = (instr >> 9) & 0x7;
                uint16_t baseR = (instr >> 6) & 0x7;
                uint16_t offset = sign_extend(instr & 0x3F, 6);

                reg[dr] = mem_read(reg[baseR] + offset);
                update_flags(reg[dr]); 

                break;
            case OP_LEA:
                /* LEA goes here */
                uint16_t dr = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);

                dr = reg[R_PC] + pc_offset;
                update_flags(reg[dr]);
                break;
            case OP_ST:
                /* ST goes here*/
                uint16_t sr = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
                mem_write(reg[sr], reg[R_PC] + pc_offset);
                break;
            case OP_STI:
                /* STI goes here*/
                uint16_t sr = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);


                mem_write(mem_read(reg[R_PC] + pc_offset), reg[sr]);
                break;
            case OP_STR:
                /* STR goes here*/
                uint16_t sr = (instr >> 9) & 0x7;
                uint16_t baseR = (instr >> 6) & 0x7;
                uint16_t offset = sign_extend(instr & 0x3F, 6);

                mem_write(reg[sr], reg[baseR] + offset);
                break;
            case OP_TRAP:
                /* TRAP goes here*/
                reg[R_R7] = reg[R_PC];
                uint16_t trapvect8 = instr & 0xFF;
                reg[R_PC] = mem_read(trapvect8);
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
}

uint16_t sign_extend(uint16_t x, int bit_count)
{
    if ((x >> (bit_count - 1)) & 1) {
        x |= (0xFFFF << bit_count);
    }
    return x;
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