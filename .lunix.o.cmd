savedcmd_/home/user/shared/lunix-tng-helpcode-20241024/lunix.o := ld -m elf_x86_64 -z noexecstack --no-warn-rwx-segments   -r -o /home/user/shared/lunix-tng-helpcode-20241024/lunix.o @/home/user/shared/lunix-tng-helpcode-20241024/lunix.mod  ; ./tools/objtool/objtool --hacks=jump_label --hacks=noinstr --hacks=skylake --ibt --orc --retpoline --rethunk --static-call --uaccess --prefix=16  --link  --module /home/user/shared/lunix-tng-helpcode-20241024/lunix.o

/home/user/shared/lunix-tng-helpcode-20241024/lunix.o: $(wildcard ./tools/objtool/objtool)
