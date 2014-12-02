// Code written by Chris Stiteler for CS472 (thursday)
// Project 3 - Due 12/04/2014
package main

import (
	"fmt"
	"strings"
)

// Memory
type Memory [1024]byte

var Main_Memory Memory

// Normal Registers
type Registers [32]int

var Regs Registers

// "program counter" value (index into instruction cache)
var PC int

// our raw hex instruction input
var InstructionCache = []uint32{}

// the Disassembleable interface is fulfilled by
// both R and I format MIPS instructions, implicitly
type Disassembleable interface {
	disassemble()
}

// Pipeline Registers
type IFIDReg struct {
	// read or write?
	isWriteReg bool

	// this is the instruction itself
	// disassembleable is the interface type for all instructions
	//Inst Disassembleable
	Inst uint32
}

// our two IFID Registers
var W_IFID IFIDReg
var R_IFID IFIDReg

// our ID/EX register, written to by ID stage
type IDEXReg struct {
	// read or write?
	isWriteReg bool

	//controls:
	RegDst   bool
	ALUSrc   bool
	ALUOp    bool
	MemRead  bool
	MemWrite bool
	Branch   bool
	MemToReg bool
	RegWrite bool

	// values
	ReadReg1Value uint32
	ReadReg2Value uint32
	SEOffset      int16

	WriteRegNum1 int
	WriteRegNum2 int
}

// our two IDEX Registers
var W_IDEX IDEXReg
var R_IDEX IDEXReg

type EXMEMReg struct {
	isWriteReg bool

	// controls:
	MemRead  bool
	MemWrite bool
	Branch   bool
	MemToReg bool
	RegWrite bool

	// values
	// CalcBTA branch target offset?
	isZero      bool
	ALUResult   int32
	SWValue     uint32
	WriteRegNum uint32
}

// our two EXMEM Registers
var W_EXMEM EXMEMReg
var R_EXMEM EXMEMReg

type MEMWBReg struct {
	isWriteReg bool

	// controls:
	MemToReg bool
	RegWrite bool

	// should this be load byte?
	LWDataValue byte
	ALUResult   int32
	WriteRegNum uint32
}

var W_MEMWB MEMWBReg
var R_MEMWB MEMWBReg

func main() {
	initialize()

	// while not done
	for i := 0; i < len(InstructionCache); i++ {
		IF_Stage()
		ID_Stage()
		EX_Stage()
		MEM_Stage()
		WB_Stage()
		print()
		advanceRegisters()
	}

}

func initialize() {
	// init main memory
	initMainMemory()

	// init pipeline registers
	initPipelineRegs()

	// init normal registers
	initRegs()

	// init instructions
	initInstructionCache()
}

func IF_Stage() {
	//fmt.Println(W_IFID)

	// simply put the current instruction in the WRITE side IF/ID reg
	W_IFID.Inst = InstructionCache[PC]
	PC++

	//fmt.Println(W_IFID)
}

func ID_Stage() {

}

func EX_Stage() {

}

func MEM_Stage() {

}

func WB_Stage() {

}

func print() {
	// print registers
	fmt.Println(Registers)

	// read/write versions of each PipeReg

}

func advanceRegisters() {
	// copy all write registers to their read counterparts

}

func initMainMemory() {
	inc := byte(0x00)
	for i, _ := range Main_Memory {
		Main_Memory[i] = inc
		if inc == 0xFF {
			inc = 0x00
			continue
		}
		inc++
	}
	fmt.Println(Main_Memory)
}

func initPipelineRegs() {
	// all control signals by default are false, therefore nops

	// init the write versions
	writeInstruction := true
	W_IFID = IFIDReg{isWriteReg: writeInstruction}
	W_IDEX = IDEXReg{isWriteReg: writeInstruction}
	W_EXMEM = EXMEMReg{isWriteReg: writeInstruction}
	W_MEMWB = MEMWBReg{isWriteReg: writeInstruction}

	// init the read versions
	writeInstruction = false
	R_IFID = IFIDReg{isWriteReg: writeInstruction}
	R_IDEX = IDEXReg{isWriteReg: writeInstruction}
	R_EXMEM = EXMEMReg{isWriteReg: writeInstruction}
	R_MEMWB = MEMWBReg{isWriteReg: writeInstruction}
}

func initRegs() {
	// ensure register 0 contains 0x0
	Regs[0] = 0x0

	// then every other register
	for i := 1; i < 32; i++ {
		Regs[i] = 0x100 + i
	}
	fmt.Println("Regs: ")
	fmt.Println(Regs)
}

func initInstructionCache() {
	// our input instructions
	InstructionCache = []uint32{
		0xA1020000,
		0x810AFFFC,
		0x00831820,
		0x01263820,
		0x01224820,
		0x81180000,
		0x81510010,
		0x00624022,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
	}

	// start "program counter" at 0
	PC = 0
}

// from Disassemble

// rInstruction represents an r-format MIPS instruction
type rInstruction struct {
	bits        uint32
	address     uint32
	instruction string
	opcode      uint32
	src1        uint32
	src2        uint32
	dest        uint32
	function    uint32
}

// iInstruction represents an i-format MIPS instruction
type iInstruction struct {
	bits        uint32
	address     uint32
	instruction string
	opcode      uint32
	src1        uint32
	src2        uint32
	constant    int16
}

type Mask struct {
	bits  uint32
	shift uint8
}

// important instruction masks and their subsequent shift values
var opcodeMask = Mask{0xFC000000, 26}
var src1Mask = Mask{0x03E00000, 21}
var src2Mask = Mask{0x001F0000, 16}
var rDestMask = Mask{0x0000F800, 11}
var rFuncMask = Mask{0x0000003F, 0}
var iConstMask = Mask{0x0000FFFF, 0}

// TODO: manipulate to a single decode, not whole array, to do one one at a time.
// TODO: add support for nop if necessary
/*
func buildInstructions(input []uint32) []Disassembleable {
	instructions := make([]Disassembleable, len(input))

	// need to get opcode, if 000000, build an rInstruction, else iInstruction
	for i, inputBits := range input {
		var newInstruction Disassembleable
		opcode := maskAndShift(opcodeMask, inputBits)

		if opcode == 0 {
			newInstruction = &rInstruction{bits: inputBits, opcode: opcode,
				address: pc}
		} else {
			newInstruction = &iInstruction{bits: inputBits, opcode: opcode,
				address: pc}
		}

		instructions[i] = newInstruction
	}

	return instructions
}
*/

// r.disassemble() extracts data from rInstruction bits
// also fulfills the Disassembleable interface for rInstructs.
func (r *rInstruction) disassemble() {
	r.src1 = maskAndShift(src1Mask, r.bits)
	r.src2 = maskAndShift(src2Mask, r.bits)
	r.dest = maskAndShift(rDestMask, r.bits)
	r.function = maskAndShift(rFuncMask, r.bits)
	r.instruction = r.getInstruction()
}

// i.dissassemble() extracts data from iInstruction bits
// also fulfills the Disassembleable interface for iInstructs.
func (i *iInstruction) disassemble() {
	i.src1 = maskAndShift(src1Mask, i.bits)
	i.src2 = maskAndShift(src2Mask, i.bits)
	i.constant = maskAndShiftShort(iConstMask, int16(i.bits))
	i.instruction = i.getInstruction()
}

func (r *rInstruction) getInstruction() string {
	switch r.function {
	case 0x20:
		return fmt.Sprintf("add")
	case 0x22:
		return fmt.Sprintf("sub")
	case 0x0:
		return fmt.Sprintf("nop")
	default:
		return ""
	}
}

func (i *iInstruction) getInstruction() string {
	switch i.opcode {
	case 0x20:
		return fmt.Sprintf("lb")
	case 0x28:
		return fmt.Sprintf("sb")
	default:
		return ""
	}
}

// getBranchToAddress() calcs branch address using pc-relative offset
// and the address of the current instruction
/*
func (i *iInstruction) getBranchToAddress() uint32 {
	// shift offset/const 2 bits left to decompress, and account for incrememted pc
	return i.address + addressSize + (uint32(i.constant) << 2)
}
*/

// maskAndShift() returns desired bits in a 32-bit value
// depending on the mask (including a shift value)
func maskAndShift(mask Mask, inputBits uint32) uint32 {
	return (inputBits & mask.bits) >> mask.shift
}

// maskAndShift() returns desired bits in a 16-bit value
// depending on the mask (including a shift value)
func maskAndShiftShort(mask Mask, inputBits int16) int16 {
	return (inputBits & int16(mask.bits)) >> mask.shift
}

// to String Methods:
func (r Registers) String() string {
	registerStrings := make([]string, len(r))
	for i, _ := range r {
		registerStrings[i] = fmt.Sprintf("[$%.2d]: 0x%.3X\n", i, r[i])
	}
	return fmt.Sprintf(strings.Join(registerStrings, ""))
}

func (m Memory) String() string {
	memoryStrings := make([]string, len(m))
	for i, _ := range m {
		if m[i] == 0xFF {
			memoryStrings[i] = fmt.Sprintf("0x%X\n\n", m[i])
		} else {
			memoryStrings[i] = fmt.Sprintf("0x%X", m[i])
		}
	}
	return fmt.Sprintf(strings.Join(memoryStrings, " "))
}
