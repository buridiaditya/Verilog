.data 								#####  data for storing strings ######
	space:      .asciiz  "    " 
	next_line:   .asciiz  "\n"
	m_msg:  	.asciiz "Enter m\n"
	n_msg:  	.asciiz "Enter n\n"
	s_msg:  	.asciiz "Enter s\n"
	input_msg:  .asciiz "Enter three positive integers m,n and s:\n"
	print_msgA:	.asciiz "the matrix is:\n"
	print_trans : .asciiz "\nthe matrix is transposed ...\n"
	
.text
main:	
	addi $sp $sp -4  		      #give 4 bytes to the stack to store the frame pointer
	sw   $fp 0($sp)  		      #store the old frame pointer
	move $fp $sp     		      #exchange the frame and stack pointers
	addi $sp $sp -12			  #allocate space for m,n,s and return address
	
	li $v0 , 4				      #display msg
	la $a0 ,input_msg			  #loads print string syscall command
	syscall						  #makes a system call
	
	li $v0 , 4				      #display msg for m
	la $a0 ,m_msg			      #loads print string syscall command
	syscall						  #makes a system call
	
	li $v0 , 5					  #loads system command to read m
	syscall						  #makes a system call
	sw $v0 , -4($fp)			  #stores m
	
	li $v0 , 4				      #display msg for n
	la $a0 ,n_msg			      #loads print string syscall command
	syscall						  #makes a system call
	
	li $v0 , 5					  #loads system command to read n
	syscall						  #makes a system call
	sw $v0 , -8($fp)			  #stores n
	
	li $v0 , 4				      #display msg for s
	la $a0 ,s_msg			      #loads print string syscall command
	syscall						  #makes a system call
	
	li $v0 , 5					  #loads system command to read s
	syscall						  #makes a system call
	sw $v0 , -12($fp)			  #stores s
	
	sub $t0 , $fp , 4			  #location of m
	sub $t1 , $fp , 8			  #location of n
	lw  $t2 , ($t0)				  #value of m
	lw  $t3 , ($t1)				  #value of n
	mul $s4 , $t2 , $t3			  #value of m * n
	sub $s5 , $s4 , 1			  # m* n -1
	mul $s3 , $s4 , 4			  # 4 * m * n
	
	sub $sp $sp $s3				  #allocate space for array A
	sub $sp $sp $s3				  #allocate space for array B
	
	sub $t1 , $fp , 12			  #location of s
	lw $t2 , ($t1) 				  #value of s is stored in t2
	sub $t1 , $fp , 16			  #location of A[0][0]
	sw  $t2 , ($t1)				  #stor the value of seed at A[0][0]
	
	li  $s0 , 330				  #store a
	li  $s1 , 100				  #store c
	li  $s2 , 481				  #store m
	
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
	sub $t0 , $fp , 4			  #location of m
	sub $t1 , $fp , 8			  #location of n
	lw  $a0 , ($t0)				  #value of m
	lw  $a1 , ($t1)				  #value of n

	sub  $a2 , $fp , 16           #address of array
	jal matPrint				  #jump and link to to the subroutine. address of next instruction is stored in $ra

	li $v0 , 4				      #display msg
	la $a0 ,print_trans			  #loads print string syscall command
	syscall						  #makes a system call

	sub $t0 , $fp , 4			  #location of m
	sub $t1 , $fp , 8			  #location of n
	lw  $a0 , ($t0)				  #value of m
	lw  $a1 , ($t1)				  #value of n

	mul $t2 , $a0 , $a1
	mul $t2 , $t2 , 4

	sub  $a2 , $fp , 16           #address of array
	sub  $a3 , $a2 , $t2

	jal matTrans

	sub $t0 , $fp , 8			  #location of n
	sub $t1 , $fp , 4			  #location of m
	lw  $a0 , ($t0)				  #value of n
	lw  $a1 , ($t1)				  #value of m

	mul $t2 , $a0 , $a1
	mul $t2 , $t2 , 4

	sub  $t3 , $fp , 16           #address of array
	sub  $a2 , $t3 , $t2

	jal matPrint


	b exit						  #branch to exit after function call


matPrint: 				############subroutine to print matrix. parameters : number of rows,number of columns, address of the array###########
	
	sub $sp , $sp , 32			  #create some space to store the variables
	sw $a0 , 0($sp)               ###
	sw $a1 , 4($sp)				  #	save the paramters
	sw $a2 , 8($sp)               ###

	li $v0 , 4				      #display msg space
	la $a0 ,next_line			  #loads print string syscall command
	syscall						  #makes a system call

	li $v0 , 4				      #display msg for matrix A
	la $a0 ,print_msgA			  #loads print string syscall command
	syscall						  #makes a system call

	lw $t0 , 0($sp) 			  #load the value of no of rows
    mul $t0 , $t0 , $a1			  #m * n is stored in $t0

    move $t1 , $a2				  #move the address into $t1

loop_print:
	beqz $t0 , return_label       #if counter is 0, go to return label  
	div $t0 , $a1				  #divide the counter by no of columns . mod is stored in hi.
	mfhi $t2
	beqz $t2 , nextline 		  #when the value is 0, that means we have printed a complete row. so print a newline
	b cont
	

nextline:
	li $v0 , 4				      #display msg space
	la $a0 ,next_line			  #loads print string syscall command
	syscall						  #makes a system call

cont:
	lw $a0 , ($t1)				 #load the value at that location
	li $v0 , 1					 #loads print int syscall command
	syscall

	li $v0 , 4				      #display msg space
	la $a0 ,space			      #loads print string syscall command
	syscall						  #makes a system call

	sub $t0 , $t0 , 1 			 #decrement the counter
	sub $t1 , $t1 , 4			 #move tot he next element of the array in row major form
    b loop_print


return_label :
	li $v0 , 4				      #display msg space
	la $a0 ,next_line			  #loads print string syscall command
	syscall						  #makes a system call

	li $v0 , 4				      #display msg space
	la $a0 ,next_line			  #loads print string syscall command
	syscall						  #makes a system call

	add $sp , $sp , 32			  #clear  the space created
	jr $ra 						  #jump back to the address in ra

matTrans:  ###### parameters : $a0 : rows ; $a1 : columns ; $a2 : present array address ; $a3 : new array address
	sub $sp , $sp , 32			  #create some space to store the variables
	sw $a0 , 0($sp)               ###
	sw $a1 , 4($sp)				  #	save the paramters
	sw $a2 , 8($sp)               #
	sw $a3 , 12($sp)              ###

	li $s0 , -1					  #initialize i with -1
	li $s3 , 0
	b outer_trans_loop

inner_trans_loop:
	mul $t0 , $a0 , $a1
	bge $s1 , $t0 , outer_trans_loop 
	mul $t0 , $s1 , 4
	sub $t0 , $a2 , $t0
	lw $t1 , ($t0)
	sub $t2 , $a3 , $s3 
	sw $t1 , ($t2)
	add $s3 , $s3 , 4
	add $s1 , $s1 , $a1
	b inner_trans_loop

set_j:
	move $s1 , $s0
	b inner_trans_loop

outer_trans_loop:
	add $s0 , $s0 , 1             #i++
	blt $s0 , $a1 , set_j

return_trans_label:
	add $sp , $sp , 32			  #clear  the space created
	jr $ra 						  #jump back to the address in ra
		
exit: 								#### exit the program ####
	li $v0 , 10 					#load exit command into $v0
	syscall 						#makes a system call
	
	
	
	
	
	
	
	
	
	
	
	 			