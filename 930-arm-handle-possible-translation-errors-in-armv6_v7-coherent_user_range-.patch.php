diff --git a/arch/arm/mm/cache-v6.S b/arch/arm/mm/cache-v6.S
index 8f5c13f..295e25d 100644
--- a/arch/arm/mm/cache-v6.S
+++ b/arch/arm/mm/cache-v6.S
@@ -12,6 +12,7 @@
 #include <linux/linkage.h>
 #include <linux/init.h>
 #include <asm/assembler.h>
+#include <asm/unwind.h>
 
 #include "proc-macros.S"
 
@@ -121,11 +122,13 @@ ENTRY(v6_coherent_kern_range)
  *	- the Icache does not read data from the write buffer
  */
 ENTRY(v6_coherent_user_range)
-
+ UNWIND(.fnstart		)
 #ifdef HARVARD_CACHE
 	bic	r0, r0, #CACHE_LINE_SIZE - 1
-1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D line
+1:
+ USER(	mcr	p15, 0, r0, c7, c10, 1	)	@ clean D line
 	add	r0, r0, #CACHE_LINE_SIZE
+2:
 	cmp	r0, r1
 	blo	1b
 #endif
@@ -143,6 +146,19 @@ ENTRY(v6_coherent_user_range)
 	mov	pc, lr
 
 /*
+ * Fault handling for the cache operation above. If the virtual address in r0
+ * isn't mapped, just try the next page.
+ */
+9001:
+	mov	r0, r0, lsr #12
+	mov	r0, r0, lsl #12
+	add	r0, r0, #4096
+	b	2b
+ UNWIND(.fnend		)
+ENDPROC(v6_coherent_user_range)
+ENDPROC(v6_coherent_kern_range)
+
+/*
  *	v6_flush_kern_dcache_page(kaddr)
  *
  *	Ensure that the data held in the page kaddr is written back
diff --git a/arch/arm/mm/cache-v7.S b/arch/arm/mm/cache-v7.S
index bda0ec3..e1bd975 100644
--- a/arch/arm/mm/cache-v7.S
+++ b/arch/arm/mm/cache-v7.S
@@ -13,6 +13,7 @@
 #include <linux/linkage.h>
 #include <linux/init.h>
 #include <asm/assembler.h>
+#include <asm/unwind.h>
 
 #include "proc-macros.S"
 
@@ -153,13 +154,16 @@ ENTRY(v7_coherent_kern_range)
  *	- the Icache does not read data from the write buffer
  */
 ENTRY(v7_coherent_user_range)
+ UNWIND(.fnstart		)
 	dcache_line_size r2, r3
 	sub	r3, r2, #1
 	bic	r0, r0, r3
-1:	mcr	p15, 0, r0, c7, c11, 1		@ clean D line to the point of unification
+1:
+ USER(	mcr	p15, 0, r0, c7, c11, 1	)	@ clean D line to the point of unification
 	dsb
-	mcr	p15, 0, r0, c7, c5, 1		@ invalidate I line
+ USER(	mcr	p15, 0, r0, c7, c5, 1	)	@ invalidate I line
 	add	r0, r0, r2
+2:
 	cmp	r0, r1
 	blo	1b
 	mov	r0, #0
@@ -167,6 +171,17 @@ ENTRY(v7_coherent_user_range)
 	dsb
 	isb
 	mov	pc, lr
+
+/*
+ * Fault handling for the cache operation above. If the virtual address in r0
+ * isn't mapped, just try the next page.
+ */
+9001:
+	mov	r0, r0, lsr #12
+	mov	r0, r0, lsl #12
+	add	r0, r0, #4096
+	b	2b
+ UNWIND(.fnend		)
 ENDPROC(v7_coherent_kern_range)
 ENDPROC(v7_coherent_user_range)
 


