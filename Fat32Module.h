#ifndef FAT32_MODULE_H
#define FAT32_MODULE_H


/// ----------------------------------------------------------------------------------------------------------------
/// FAT32 PART
/// ----------------------------------------------------------------------------------------------------------------

/// File system object structure
typedef struct _FATFS_ {
  uint8_t csize;      // Number of sectors per cluster
  uint8_t csect;      // File sector address in the cluster
  uint32_t max_clust; // Maximum cluster# + 1. Number of clusters is max_clust - 2
  uint32_t fatbase;   // FAT start sector
  uint32_t dirbase;   // Root directory start Cluster
  uint32_t database;  // Data start sector
  uint32_t fptr;      // File R/W pointer
  uint32_t fsize;     // File size
  uint32_t org_clust; // File start cluster
  uint32_t curr_clust;// File current cluster
  uint32_t dsect;     // File current data sector
  uint8_t* buf;       // temporary Buffer for directory
} FATFS;


/// Directory object structure
typedef struct _DIR_ {
  uint16_t  index;    // Current read/write index number
  uint8_t*  fn;       // Pointer to the SFN (in/out) {file[8],ext[3],status[1]}
  uint32_t sclust;    // Table start cluster (0:Static table)
  uint32_t clust;     // Current cluster
  uint32_t  sect;     // Current sector
} DIR;


/// File status structure
typedef struct _FILINFO_ {
  uint32_t  fsize;    // File size
  uint16_t  fdate;    // Last modified date
  uint16_t  ftime;    // Last modified time
  uint8_t fattrib;    // Attribute
  char fname[13];    /* File name */
} FILINFO;


/// File function return code (FRESULT)
typedef enum {
  FR_OK = 0,          // 0
  FR_DISK_ERR,        // 1
  FR_NOT_READY,       // 2
  FR_NO_FILE,         // 3
  FR_NO_PATH,         // 4
  FR_INVALID_NAME,    // 5
  FR_STREAM_ERR,      // 6
  FR_INVALID_OBJECT,  // 7
  FR_NOT_ENABLED,     // 8
  FR_NO_FILESYSTEM,   // 9
} FRESULT;


/// File attribute bits for directory entry
#define AM_RDO  0x01  // Read only
#define AM_HID  0x02  // Hidden
#define AM_SYS  0x04  // System
#define AM_VOL  0x08  // Volume label
#define AM_LFN  0x0F  // LFN entry
#define AM_DIR  0x10  // Directory
#define AM_ARC  0x20  // Archive
#define AM_MASK 0x3F  // Mask of defined bits

/// Offsets in FAT structure -13
#define BPB_SecPerClus  0 // 0x0d - 13 (1 byte)  Logical sectors per cluster
#define BPB_RsvdSecCnt  1 // 0x0e - 13 (2 bytes) Count of reserved logical sectors
#define BPB_NumFATs     3 // 0x10 - 13 (1 byte)  Number of File Allocation Tables
#define BPB_TotSec32   19 // 0x20 - 13 (4 bytes) Total logical sectors
#define BPB_FATSz32    23 // 0x24 - 13 (4 bytes) Logical sectors per file allocation table
#define BPB_RootClus   31 // 0x2c - 13 (4 bytes) Cluster number of root directory start, typically 2
#define MBR_Table     446 // 0x1cb -13

/// Offsets in directory structure
#define DIR_Name        0
#define DIR_Attr       11
#define DIR_NTres      12
#define DIR_CrtTime    14
#define DIR_CrtDate    16
#define DIR_FstClusHI  20
#define DIR_WrtTime    22
#define DIR_WrtDate    24
#define DIR_FstClusLO  26
#define DIR_FileSize   28


uint32_t get_fat (uint32_t);
FRESULT pf_mount (FATFS*);				/* Mount/Unmount a logical drive */
uint8_t pf_open (const char*);			/* Open a file */
uint16_t pf_read (void*, uint16_t);	/* Read data from a file */
FRESULT pf_lseek (uint32_t);				/* Move file pointer of a file object */
FRESULT pf_opendir (DIR*, const char*);	/* Open an existing directory */
FRESULT pf_readdir (DIR*, FILINFO*, uint8_t);	/* Read a directory item */
FRESULT pf_dirnext (DIR*);
FRESULT dir_rewind (DIR*);
FRESULT pf_dirprev (DIR*);

#endif /* FAT32_MODULE_H */
