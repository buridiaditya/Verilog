.data
	new_line: .asciiz "\n"
	prompt1: .asciiz "The matrix passed on this invocation is :"
	input_msg:	.asciiz "Enter the order of the square matrix whose determinant is to be found\n"
	seed_msg:	.asciiz "Enter some positive integer for the value of the seed s:\n"
	space:	.asciiz  "    " 
	result: .asciiz "Finally the determinant is : "
.text
main: 	
	addi $sp $sp -4  		      #give 4 bytes to the stack to store the frame pointer
	sw   $fp 0($sp)  		      #store the old frame pointer
	move $fp $sp     		      #exchange the frame and stack pointers
	addi $sp $sp -8			      #allocate space for n,s
	
	li $v0 , 4				      #display msg
	la $a0 ,input_msg			  #loads print string syscall command
	syscall						  #makes a system call
	
	li $v0 , 5					  #loads system command to read n
	syscall						  #makes a system call
	sw $v0 , -4($fp)			  #stores n
	
	
	li $v0 , 4				      #display msg for s
	la $a0 ,seed_msg			  #loads print string syscall command
	syscall						  #makes a system call
	
	li $v0 , 5					  #loads system command to read s
	syscall						  #makes a system call
	sw $v0 , -8($fp)			  #stores s
	
	sub $t0 , $fp , 4			  #location of n
	lw  $t2 , ($t0)				  #value of n
	mul $s4 , $t2 , $t2			  #value of n * n
	sub $s5 , $s4 , 1			  # n* n -1
	mul $s3 , $s4 , 4			  # 4 * n * n
	
	sub $sp $sp $s3				  #allocate space for array A
	
	sub $t1 , $fp , 8			  #location of s
	lw $t2 , ($t1) 				  #value of s is stored in t2
	sub $t1 , $fp , 12			  #location of A[0][0]
	sw  $t2 , ($t1)				  #stor the value of seed at A[0][0]
	
	li  $s0 , 330				  #store a
	li  $s1 , 100				  #store c
	li  $s2 , 7				  #store m
	
random_gen:
	beqz $s5 , func_call		 #after loading the values make a function call to print

	lw $t3 , ($t1)				  #present element(X)
	mul $t3 , $t3 , $s0			  #aX
	add $t3 , $t3 , $s1			  #ax+c
	div $t3 , $s2				  #rem is in hi
	sub $t2 , $t1 , 4			  #location of next element
	mfhi $t3
	sw $t3 , ($t2)				  #aX+c mod m is stored in next element

	sub $t1 , $t1 , 4			  #t1 points present element now
	sub $s5 , $s5 , 1 			  #decrements the counter
	b random_gen
	
func_call:
	sub $t0 , $fp , 4			  #location of n
	lw  $a0 , ($t0)				  #value of n
	move  $a1 , $sp           		#address of array
	jal findDet				  #jump and link to to the subroutine. address of next instruction is stored in $ra			  
	move $t0, $v0		
	la $a0, result			# load String in result
	li $v0, 4				# set to print_string
	syscall
	move $a0, $t0			# load the determinant
	li $v0, 1				# set to print_int
	syscall
	la $a0, new_line		# load String in new_line
	li $v0, 4				# set to print_int
	syscall
	b exit						  #branch to exit after function call

findDet: 
	sub $sp, $sp, 8	# Create 8 bytes space on stack
	sw $a0, ($sp)	# Save 1st parameter
	sw $ra, 4($sp)	# Save return address
	la $a0, prompt1	#Set parameter 1 to be address of prompt1 data 
	li $v0, 4	#Set to print_string
	syscall	#System call
	lw $a0, ($sp)	# Restore parameter 1 from stack
	jal sqMatPrint #Jump to Subroutine sqMatPrint storing the return address in $ra
	li $v0, 0	# initialize it to 0
	lw $ra, 4($sp)	# Return address is restored
	add $sp, $sp, 8	# Delete Stack
findDetInner:									
	beq $a0, 2, twoNumDet	#If no of rows == 2 execute the subroutine to compute Determinant of 2x2 Matrix
	li $t0, 0		#Store 0 in $t0
	sub $sp, $sp, 24	#Create space in stack for storing 6 words 
	sw $ra, 20($sp)	#Store return address
	sw $t0, 16($sp) #Store the sum computed till then
	sw $a0, 12($sp)	#Store parameter 1 i.e 'n'
	sw $a1, 8($sp)  #Store addr of Input Matrix 'addr'
	sw $t0, 4($sp)	#Store index = 0
	sw $fp, ($sp)	#Store old Frame Pointer in stack
	move $fp, $sp 	#Store Current Stack Pointer Address in Frame Pointer
	FOR_LOOP:
		lw $t0, 4($fp)	# Load the index i in $t0
		lw $t1, 8($fp)	# Load the address of given matrix in a register $t1
		lw $t2, 12($fp) # Load the value n in $t2
		beq $t0, $t2, RETURN_SUB # if i == n  then return from function call
		add $t0, $t0, 1	# i = i+1
		sw $t0, 4($fp)	# Update the counter on stack
		mul $t3, $t2, 4	#Compute 4*n
		add $t1, $t1, $t3	# Skip the first row in the given matrix
		sub $a0, $t2, 1 # set parameter 1 as n-1
		mul $t3, $a0, $a0 # compute (n-1)x(n-1)  
		mul $t3, $t3, 4	# Compute (n-1)x(n-1)x4  
		sub $sp, $sp, $t3 # Create a (n-1)x(n-1) integer array on stack
		move $t3, $sp # Store the address of the (n-1)x(n-1) integer matrix in register $t3
		li $t4, 0	# Register Represents Row
		li $t5, 0	# Register Represents Column
		FOR_LOOP_1_1:
			beq $t4, $a0, CONTINUE_1	#Termiante loop if $t0 == n-1
			add $t4, $t4, 1	#Increment Row register
			li $t5, 0	#Reset column index register to 0
			FOR_LOOP_1_2:
				add $t5, $t5, 1	# Increment column index register
				bgt $t5, $t2, FOR_LOOP_1_1	#Termiante loop if $t0 == n
				beq $t5, $t0, SKIP	#SKIP if ith column is accessed
				lw $t6, ($t1)	# Get the number pointed by 8($fp)
				sw $t6, ($t3)	#Save the value in the submatrix
				add $t3, $t3, 4	#Increment the address by 4 bytes to make it point to next element
				add $t1, $t1, 4 #Increment address to point next element in input array
				b FOR_LOOP_1_2
	CONTINUE_1:
		move $a1, $sp 			#Load Stack pointer into $a1
		jal findDetInner		#Jump function findDetInner
		li $t0, 2				# Store 2 in $t0
		li $t1, 1				# Store 1 in $t1
		lw $t3, 4($fp)			# Load the current column number 
		div $t3, $t0 			# Divide to check if even or odd
		mfhi $t0				# Get remainder
		bnez $t0, CONTINUE_2	# Check even or odd
		mul $t1, $t1, -1		# multiply -1 * 1
		CONTINUE_2:
		mul $t1, $t1, $v0		# result x +/-(1)
		lw $t0, 8($fp)			# get address of input matrix
		sub $t3, $t3, 1
		mul $t3, $t3, 4			# 4x(i-1)
		add $t0, $t3, $t0		# pointer + 4x(i-1)
		lw $t0, ($t0)			# store value at Pointer +4*(i-1) in t0
		mul $t0, $t0, $t1		# result x (+/-1) x value
		lw $t1, 16($fp)			
		add $t1, $t1, $t0	
		sw $t1, 16($fp) 		# Store the sum
		lw $t1, 12($fp)			# Store n
		sub $t1, $t1, 1		# Compute n-1
		mul $t1, $t1, $t1	# Compute (n-1)*(n-1)
		mul $t1, $t1, 4		# Compute (n-1)*(n-1)*4
		add $sp, $sp, $t1   # Subtract Stack Pointer to delete the submatrix
		move $a0, $t0

		b FOR_LOOP	
	SKIP:
		add $t1, $t1, 4	#Move the address pointing to the element in the input matrix to point next element
		b FOR_LOOP_1_2	#Branch to FOR_LOOP_1_2
	RETURN_SUB:
		lw $ra, 20($fp)	#Load the return address
		lw $v0, 16($fp)	#Load the sum value into return register
		lw $fp, ($fp) # Restore the fp
		sub $sp, $sp, 24	# Subtract the param Varibles data from stack
		jr $ra		# Jump to return address
twoNumDet: 
	lw $t0, ($a1)	#Load A[0][0]
	lw $t1, 12($a1)	#Load A[1][1]
	mul $t3, $t0, $t1	#Multilpy A[0][0]xA[1][1]
	lw $t0, 4($a1)	#Load A[0][1]
	lw $t1, 8($a1)	#Load A[1][0]
	mul $t1, $t0, $t1	#Multilpy A[0][1]xA[1][0]
	sub $v0, $t3, $t1	# Compute Determinant
	jr $ra

sqMatPrint:
	move $t0, $a0	# Store n in i
	move $t1, $a0	# Store n in j
	move $t3, $a0	# Store n
	move $t4, $a1	# Store address
	FOR_LOOP_1:
		la $a0, new_line	# Store new_line 
		li $v0, 4	# set to print_string
		syscall		# system call
		beqz $t0, RETURN_PRINT # Return from subroutine
		sub $t0, $t0, 1		# 
		move $t1, $t3
		FOR_LOOP_2:
			beqz $t1, FOR_LOOP_1
			lw $a0, ($t4) 
			li $v0, 1
			syscall
			la $a0, space 
			li $v0, 4
			syscall
			add $t4, $t4, 4
			sub $t1, $t1, 1
			b FOR_LOOP_2
RETURN_PRINT:
	move $a0, $t3
	jr $ra	
	
exit: 								#### exit the program ####
	li $v0 , 10 					#load exit command into $v0
	syscall 						#makes a system call