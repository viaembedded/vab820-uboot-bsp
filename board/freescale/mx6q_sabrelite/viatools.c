

static int sTextAttribute = 0;
enum { taNormal=0, taBold=1, taUnderscore=4, taBlink=5};
static void text_attribute(int textAttribute)
{
 sTextAttribute = textAttribute;
}

enum { clNone=0,
clRed=31, clGreen, clYellow, clBlue, clMagenta,
clCyan=36 };
static void color_print(const char* p, int color)
{
 printf("\x1b[%d;%dm%s\x1b[0m", sTextAttribute, color, p);
}

static void show_cpu(void) {
 printf("CPU: %s, rev = 0x%x\n",
  (cpu_is_mx6q() ? "MX6Q" : "MX6DL"),
  mx6_chip_rev()
 );
}

static void show_ram(void) {
 int nRamSize = 1024;
 u32 reg = readl(GPIO2_BASE_ADDR + GPIO_PSR);

 switch ( reg&0x03 ) {
  case 1: nRamSize = 3840; break;
  case 2: nRamSize = (2 * 1024 ); break;
  default: nRamSize = (1 * 1024); break;
 }
 printf("RAM: size = %d MB\n", nRamSize);
}

static void show_samsung_emmc_info(struct mmc *mmc) {
 printf("Samsung, ");
 switch (mmc->cid[1]) {
  case 0x34473146: /* 1st ID = 0x4d */
   printf("eMMC 4.4, 4GB\n"); break;
  case 0x594d4433: /* 1st ID = 0x34 */
   printf("eMMC 5.0, 4GB\n"); break;
  case 0x38473147: /* 1st ID = 0x4d */
   printf("eMMC 5.0, 8GB\n"); break;
  default:
   printf("Unknown Product Name = 0x%08x\n", mmc->cid[1]);
 }
}

static void show_hynix_emmc_info(struct mmc *mmc) {
 printf("Hynix, ");
 switch (mmc->cid[1]) {
  case 0x38473264: /* 1st ID = 0x48 */
   printf("eMMC 4.4, 8GB\n"); break;
  default:
   printf("Unknown Product Name = 0x%08x\n", mmc->cid[1]);
 }
}

static void show_emmc(void) {
 struct mmc *mmc = find_mmc_device(1);
 mmc_init(mmc);
 printf("eMMC: ");
 switch (mmc->cid[0] >> 24) {
  case 0x15: show_samsung_emmc_info(mmc); break;
  case 0x90: show_hynix_emmc_info(mmc); break;
  default:
   printf("Unknown Manufacturer ID = 0x%x, Product Name = 0x%08x\n"
    , mmc->cid[0] >> 24, mmc->cid[1]); break;
 }
}

static void show_spi(void) {
 printf("SPI ROM: ");
 run_command("sf probe 1", 0);
}

static void show_hw_info(void)
{
 printf("\n\n");
 show_cpu();
 show_ram();
 show_emmc();
 show_spi();
 printf("\n\n");
}

int do_viatools(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
 int ch;
 printf("\n\n");
 while ( 1 ) {
  text_attribute(taBold);
  color_print("VIA tools\n", clCyan);
  text_attribute(taNormal);
  printf("===================================\n");
  printf("1. hardware information\n");
  printf("others. exit\n");
  printf("===================================\n");
  printf("choose : ");
  ch = getc(); printf("%c\n", ch);
  //printf("your input = %c, 0x%x\n", ch, ch);
  switch (ch) {
   case '1': show_hw_info(); break;
   default: return 0;
  }
 }
 return 0;
}

U_BOOT_CMD(
	viatools, CONFIG_SYS_MAXARGS, 0, do_viatools,
	"VIA tools",
	"- some useful tools"
);


