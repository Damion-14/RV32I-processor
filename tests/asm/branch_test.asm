.text
main:
  #-------------------------------------------------------------
  # Branch instruction tests
  #-------------------------------------------------------------

  #-------------------------------------------------------------
  # Test BEQ (Branch if Equal)
  #-------------------------------------------------------------

test_2:
  li x1, 0x00000005
  li x2, 0x00000005
  li gp, 2
  bne x1, x2, fail    # Should not branch (they're equal)
  beq x1, x2, test_3  # Should branch (they're equal)
  j fail              # Should not reach here

test_3:
  li x1, 0x00000005
  li x2, 0x00000006
  li gp, 3
  beq x1, x2, fail    # Should not branch (they're not equal)

  #-------------------------------------------------------------
  # Test BNE (Branch if Not Equal)
  #-------------------------------------------------------------

test_4:
  li x1, 0x00000007
  li x2, 0x00000008
  li gp, 4
  beq x1, x2, fail    # Should not branch (they're not equal)
  bne x1, x2, test_5  # Should branch (they're not equal)
  j fail              # Should not reach here

test_5:
  li x1, 0x0000000A
  li x2, 0x0000000A
  li gp, 5
  bne x1, x2, fail    # Should not branch (they're equal)

  #-------------------------------------------------------------
  # Test BLT (Branch if Less Than - signed)
  #-------------------------------------------------------------

test_6:
  li x1, -5           # 0xFFFFFFFB
  li x2, 10
  li gp, 6
  bge x1, x2, fail    # -5 < 10, should not branch
  blt x1, x2, test_7  # -5 < 10, should branch
  j fail

test_7:
  li x1, 10
  li x2, -5
  li gp, 7
  blt x1, x2, fail    # 10 > -5, should not branch

test_8:
  li x1, 5
  li x2, 5
  li gp, 8
  blt x1, x2, fail    # 5 == 5, should not branch

  #-------------------------------------------------------------
  # Test BGE (Branch if Greater or Equal - signed)
  #-------------------------------------------------------------

test_9:
  li x1, 10
  li x2, -5
  li gp, 9
  blt x1, x2, fail    # 10 > -5, should not branch
  bge x1, x2, test_10 # 10 >= -5, should branch
  j fail

test_10:
  li x1, 5
  li x2, 5
  li gp, 10
  blt x1, x2, fail    # 5 == 5, should not branch
  bge x1, x2, test_11 # 5 >= 5, should branch
  j fail

test_11:
  li x1, -10
  li x2, 5
  li gp, 11
  bge x1, x2, fail    # -10 < 5, should not branch

  #-------------------------------------------------------------
  # Test BLTU (Branch if Less Than Unsigned)
  #-------------------------------------------------------------

test_12:
  li x1, 5
  li x2, 10
  li gp, 12
  bgeu x1, x2, fail   # 5 < 10 (unsigned), should not branch
  bltu x1, x2, test_13 # 5 < 10 (unsigned), should branch
  j fail

test_13:
  li x1, 0xFFFFFFFB  # Large unsigned number
  li x2, 10
  li gp, 13
  bltu x1, x2, fail   # 0xFFFFFFFB > 10 (unsigned), should not branch

test_14:
  li x1, 5
  li x2, 5
  li gp, 14
  bltu x1, x2, fail   # 5 == 5, should not branch

  #-------------------------------------------------------------
  # Test BGEU (Branch if Greater or Equal Unsigned)
  #-------------------------------------------------------------

test_15:
  li x1, 10
  li x2, 5
  li gp, 15
  bltu x1, x2, fail   # 10 > 5 (unsigned), should not branch
  bgeu x1, x2, test_16 # 10 >= 5 (unsigned), should branch
  j fail

test_16:
  li x1, 5
  li x2, 5
  li gp, 16
  bltu x1, x2, fail   # 5 == 5, should not branch
  bgeu x1, x2, test_17 # 5 >= 5 (unsigned), should branch
  j fail

test_17:
  li x1, 0xFFFFFFFB  # Large unsigned number
  li x2, 10
  li gp, 17
  bltu x1, x2, fail   # 0xFFFFFFFB > 10 (unsigned), should not branch
  bgeu x1, x2, test_18 # 0xFFFFFFFB >= 10 (unsigned), should branch
  j fail

test_18:
  li x1, 5
  li x2, 0xFFFFFFFF
  li gp, 18
  bgeu x1, x2, fail   # 5 < 0xFFFFFFFF (unsigned), should not branch

  #-------------------------------------------------------------
  # Test JAL (Jump and Link)
  #-------------------------------------------------------------

test_19:
  li gp, 19
  li x1, 0            # Clear x1
  jal x1, jal_target_1  # Jump and save return address in x1
  # x1 should now contain the address of this instruction (next after jal)
  li x2, 0xBAD        # Should not execute
  j fail

jal_target_1:
  # x1 should contain address of the "li x2, 0xBAD" instruction above
  beq x1, x0, fail    # x1 should not be zero
  # Just verify jump worked by continuing to next test

test_20:
  li gp, 20
  jal x0, test_21     # Jump without saving return address (x0)
  j fail              # Should not reach here

  #-------------------------------------------------------------
  # Test JALR (Jump and Link Register)
  #-------------------------------------------------------------

test_21:
  li gp, 21
  li x4, 0            # Clear x4
  la x3, jalr_target_1  # Load target address
  jalr x4, x3, 0      # Jump to address in x3, save return in x4
  li x5, 0xBAD        # Should not execute
  j fail

jalr_target_1:
  # x4 should contain address of "li x5, 0xBAD" instruction
  beq x4, x0, fail    # x4 should not be zero
  # Just verify jump worked by continuing

test_22:
  li gp, 22
  la x6, test_23      # Load target address
  jalr x0, x6, 0      # Jump without saving return address (x0)
  j fail              # Should not reach here

test_23:
  li gp, 23
  # Test JALR with offset
  li x9, 0            # Clear x9
  la x8, jalr_target_2
  addi x8, x8, -8     # Subtract 8 from address
  jalr x9, x8, 8      # Jump to (x8 + 8), save return in x9
  j fail              # Should not reach here

jalr_target_2:
  # x9 should contain return address
  beq x9, x0, fail    # x9 should not be zero
  # Test forward branch over code
test_24:
  li gp, 24
  li x10, 0
  beq x10, x10, skip_section
  li x11, 0xBAD       # Should not execute
  j fail

skip_section:
  li x11, 0x600D      # Should execute
  li x12, 0x600D
  bne x11, x12, fail

  # Test backward branch (simple loop)
test_25:
  li gp, 25
  li x13, 0           # Counter
  li x14, 5           # Loop limit

loop_start:
  addi x13, x13, 1    # Increment counter
  blt x13, x14, loop_start  # Loop while counter < 5
  li x15, 5
  bne x13, x15, fail  # Check final counter value

  # Test nested branches
test_26:
  li gp, 26
  li x16, 10
  li x17, 20
  blt x16, x17, nested_1
  j fail

nested_1:
  li x18, 30
  bge x18, x17, nested_2
  j fail

nested_2:
  beq x16, x16, test_27
  j fail

test_27:
  li gp, 27
  # All tests passed if we reach here

pass:
	li a0, 1
	ebreak
fail:
	li a0, 0xdead
	ebreak
