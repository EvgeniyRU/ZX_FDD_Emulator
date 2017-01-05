// FAT32 Module Based on Petit FatFs
#include <stdint.h>
#include <string.h>
#include "SDCardModule.h"
#include "Fat32Module.h"

static FATFS *FatFs;      // Pointer to the file system object (logical drive)


/// Private Functions
///////////////////////////////////////////////////////////


/// Check if chr is contained in the string
static int chk_chr (const char* str, int chr)
{
  while (*str && *str != chr) str++;
  return *str;
}


/// FAT access - Read value of a FAT entry
uint32_t get_fat (      // 1:IO error, Else:Cluster status
  uint32_t clst      // Cluster# to get the link information
)
{
    uint16_t wc, bc, ofs;
    uint32_t buf;
    FATFS *fs = FatFs;

    if (clst < 2 || clst >= fs->max_clust)        // Range check
        return 1;

    if (card_readp(&buf, fs->fatbase + clst / 128, ((uint8_t)clst % 128) * 4, 4)) return 1;  
    return buf & 0x0FFFFFFF;
}

/// Get cluster LBA number
static uint32_t clust2sect (    // !=0: Sector number, 0: Failed - invalid cluster#
    uint32_t clst      // Cluster# to be converted
)
{
    FATFS *fs = FatFs;
    clst -= 2;
    if (clst >= (fs->max_clust - 2)) return 0;      // Invalid cluster#
    return clst * fs->csize + fs->database;
}


/// Rewind directory index
FRESULT dir_rewind (
  DIR *dj       // Pointer to directory object
)
{
  uint32_t clst;
  FATFS *fs = FatFs;

  dj->index = 0;
  clst = dj->sclust;
  if (clst == 1 || clst >= fs->max_clust)       // Check start cluster range
    return FR_DISK_ERR;
  if (!clst)              // Replace cluster# 0 with root cluster# if in FAT32
    clst = fs->dirbase;
  dj->clust = clst;           // Current cluster
  dj->sect = clst ? clust2sect(clst) : fs->dirbase;   // Current sector

  return FR_OK;
}


/// Move directory index next
FRESULT pf_dirnext (      // FR_OK:Succeeded, FR_NO_FILE:End of table, FR_DENIED:EOT and could not streach
  DIR *dj       // Pointer to directory object
)
{
    uint32_t clst, tmp_clst, tmp_sect;
    uint16_t i;
    uint8_t* dir;
    FATFS *fs = FatFs;
    FRESULT res;

    dir = FatFs->buf;
    i = dj->index + 1;            // dj->index is entry number in directory
    tmp_clst = dj->clust;
    tmp_sect = dj->sect;
     
    if (!i || !dj->sect)            // Report EOT when index has reached 65535
        return FR_NO_FILE;

    if (!(i % 16)) {    /* Sector changed? */
        dj->sect++;           // Jump to Next sector

        if (dj->clust == 0)
        { // Static table
            if (i >= 0)         // Report EOT when end of table
                return FR_NO_FILE;
        }
        else
        { // Dynamic table
            if (((i >> 4) & (fs->csize-1)) == 0)
            { // Cluster changed?
                clst = get_fat(dj->clust);    // Get next cluster
                if (clst <= 1) return FR_DISK_ERR;  // Unable to get cluster number
                if (clst >= fs->max_clust)    // When it reached end of clusters
                    return FR_NO_FILE;      /* Report EOT */
                dj->clust = clst;     // Initialize data for new cluster
                dj->sect = clust2sect(clst);
            }
        }
    }

    res = card_readp(dir, dj->sect, (uint16_t)((i % 16) * 32), 32)  // Read an 32 byte directory entry
         ? FR_DISK_ERR : FR_OK;
    if (res != FR_OK) return res;

    if (dir[DIR_Name] == 0) 
    {
        dj->sect = tmp_sect;
        dj->clust = tmp_clst;
        return FR_NO_FILE;    // Reached the end of directory
    }

    dj->index = i;              // index of next directory entry

    return FR_OK;
}


/// Move directory index prev
FRESULT pf_dirprev (      // FR_OK:Succeeded
    DIR *dj       // Pointer to directory object
)
{
    uint32_t clst;
    uint16_t i, tmp, cl_num;
    FATFS *fs = FatFs;
    uint8_t s_num;

    if (!fs) return FR_NOT_ENABLED;         // Check if file system is mounted

    i = dj->index;
    if (!i || !dj->sect)        // we're already at 0 index, unable to decrease
        return FR_NO_FILE;
    
    i--;

    tmp = (i >> 4);
    cl_num =  tmp / fs->csize;         // calculate cluster number for directory entry
    s_num =  tmp % fs->csize;          // calculate sector number in cluster

    clst = dj->sclust;
    if (clst == 1 || clst >= fs->max_clust)       // Check start cluster range
        return FR_DISK_ERR;
    if (!clst)              // Replace cluster# 0 with root cluster# if in FAT32
        clst = fs->dirbase;

    for(tmp = 0; tmp < cl_num; tmp++)
    { // find cluster
        clst = get_fat(clst);
        if (clst <= 1) return FR_DISK_ERR;      // Unable to get cluster number
    }
    dj->clust = clst;         // Initialize data for new cluster
    dj->sect = clust2sect(clst) + s_num;
    dj->index = i;
    
    return FR_OK;
}


/// Find an object in the directory
static FRESULT dir_find (
    DIR *dj       // Pointer to the directory object linked to the file name
)
{
    FRESULT res;
    uint8_t c, *dir;

    res = dir_rewind(dj);           // Rewind directory object
    if (res != FR_OK) return res;

    dir = FatFs->buf;
    do {
        res = card_readp(dir, dj->sect, (uint16_t)((dj->index % 16) * 32), 32)  // Read an entry
            ? FR_DISK_ERR : FR_OK;
        if (res != FR_OK) break;
        c = dir[DIR_Name];          // First character
        if (c == 0) { res = FR_NO_FILE; break; }    // Reached the end of directory
        if (!(dir[DIR_Attr] & AM_VOL) && !memcmp(dir, dj->fn, 11)) /* Is it a valid entry? */
            break;            // Found, break
        res = pf_dirnext(dj);         // Next entry if not found yet
    } while (res == FR_OK);

    return res;
}


/// Read an object from the directory
static FRESULT dir_read (
    DIR *dj,      // Pointer to the directory object to store read object name
    uint8_t move_flag
)
{
    FRESULT res;
    uint8_t a, c;
    uint8_t *dir = FatFs->buf;

    res = FR_NO_FILE;
    while (dj->sect)
    {
          res = card_readp(dir, dj->sect, (uint16_t)((dj->index % 16) * 32), 32)  // Read an 32 byte directory entry
              ? FR_DISK_ERR : FR_OK;
          if (res != FR_OK) break;
          c = dir[DIR_Name];
          if (c == 0) { res = FR_NO_FILE; break; }    // Reached the end of directory
          a = dir[DIR_Attr] & AM_MASK;
          if (c != 0xE5 && !(a & AM_VOL))  // Is it a valid entry?  && c != '.'
              break;            // FOUND, break

          if(move_flag)
          {
              res = pf_dirprev(dj);       // Try Prev entry if not found yet
              if ( res != FR_OK) break;
          }
          else
          {
              res = pf_dirnext(dj);       // Try Next entry if not found yet
              if ( res != FR_OK) break;
          }
    }

    if (res != FR_OK) dj->sect = 0;
    return res;
}


/// Pick a segment and create the object name in directory form
static FRESULT create_name (
  DIR *dj,      // Pointer to the directory object
  const char **path   // Pointer to pointer to the segment in the path string
)
{
  uint8_t c, ni, si, i, *sfn;
  const char *p;

  // Create file name in directory form
  sfn = dj->fn;
  memset(sfn, ' ', 11); // fill filename template with space symbol
  si = i = 0; ni = 8;
  p = *path;
  for (;;)
  {
    c = p[si++];
    if (c < ' ' || c == '/') break;       // End of segment
    if (c == '.' || i >= ni) {
      if (ni != 8 || c != '.') return FR_INVALID_NAME;
      i = 8; ni = 11;
      continue;
    }
    if (c >= 0x7F || chk_chr(" +,;[=\\]\"*:<>\?|", c))  // Reject unallowable chrs for SFN (Short File Name)
      return FR_INVALID_NAME;

    if (c >='a' && c <= 'z') c -= 0x20;     // convert locase to upcase (standard of SFN)

    sfn[i++] = c;
  }
  if (!i) return FR_INVALID_NAME;         // Reject null string
  *path = &p[si];             // Rerurn pointer to the next segment

  sfn[11] = (c < ' ') ? 1 : 0;          // Set last segment flag if end of path

  return FR_OK;
}


/// Get file information from directory entry
static void get_fileinfo (    // No return code
    DIR *dj,      // Pointer to the directory object
    FILINFO *fno      // Pointer to store the file information
)
{
    uint8_t i, c;
    char *p;
    uint8_t *dir = FatFs->buf;

    p = fno->fname;
    if (dj->sect)
    {
        for (i = 0; i < 8; i++)
        { // Copy file name body
            c = dir[i];
            if (c == ' ') break;
            if (c == 0x05) c = 0xE5;
            *p++ = c;
        }
        if (dir[8] != ' ')
        { // Copy file name extension
            *p++ = '.';
            for (i = 8; i < 11; i++) {
                c = dir[i];
                if (c == ' ') break;
                *p++ = c;
            }
        }
        fno->fattrib = dir[DIR_Attr];       // Attribute
        fno->fsize = (uint32_t)(*(uint32_t*)(uint8_t*)(&dir[DIR_FileSize]));    // Size
        fno->fdate = (uint16_t)(*(uint16_t*)(uint8_t*)(&dir[DIR_WrtDate]));     // Date
        fno->ftime = (uint16_t)(*(uint16_t*)(uint8_t*)(&dir[DIR_WrtTime]));     // Time
    }
    *p = 0;
}


static
uint32_t get_clust (
    uint8_t* dir   /* Pointer to directory entry */
)
{
    FATFS *fs = FatFs;
    uint32_t clst = 0;

    clst = (uint16_t)(*(uint16_t*)(&dir[DIR_FstClusHI]));
    clst <<= 16;
    clst |= (uint16_t)(*(uint16_t*)(&dir[DIR_FstClusLO]));
    return clst;
}


/// Follow a file path
static FRESULT follow_path (    // FR_OK(0): successful, !=0: error code
    DIR *dj,      // Directory object to return last directory and found object
    const char *path    // Full-path string to find a file or directory
)
{
    FRESULT res;
    uint8_t *dir;
  
    while (*path == ' ') path++;    /* Strip leading spaces */
    if (*path == '/') path++;         // Strip heading separator
    dj->sclust = 0;             // Set start directory (always root dir)

    if ((uint8_t)*path < ' ')           // Null path means the root directory
    {
        res = dir_rewind(dj);
        dir[0] = 0;
    }
    else
    {               // Follow path
        for (;;)
        {
            res = create_name(dj, &path);     // Get a file name segment
            if (res != FR_OK) break;
            res = dir_find(dj);       // Find it
            if (res != FR_OK) break;    /* Could not find the object */
            if (dj->fn[11]) break;      /* Last segment match. Function completed. */
            dir = FatFs->buf;        // There is next segment. Follow the sub directory
            if (!(dir[DIR_Attr] & AM_DIR)) {      // Cannot follow because it is a file
                res = FR_NO_FILE; break;
            }
            dj->sclust = get_clust(dir);  /* Follow next */
        }
    }

    return res;
}


/// Check a sector if it is an FAT boot record
static uint8_t check_fs (     // 0:The FAT boot record, 1:Valid boot record but not an FAT, 2:Not a boot record, 3:Error
  uint8_t *buf,     // Working buffer
  uint32_t sect     // Sector# (lba) to check if it is an FAT boot record or not
)
{
  if (card_readp(buf, sect, 0x1FE, 2))        // Read the boot sector signature
    return 3;

  if ((uint16_t)*(uint16_t*)buf != 0xAA55)         // Check MBR signature
    return 2;

  if (!card_readp(buf, sect, 0x52, 2) && (uint16_t)(*(uint16_t*)(uint8_t*)buf) == 0x4146)  // Check FAT32
    return 0;

  return 1;
}

// --------------------------------------------------------------------------------------------------------------------------

/// Mount/Unmount a Locical Drive
FRESULT pf_mount (
  FATFS *fs     // Pointer to new file system object (NULL: Unmount)
)
{
  uint8_t fmt, buf[36];
  uint32_t bsect, fsize;
  uint16_t rsvsect;

  FatFs = 0;
  if (!fs) return FR_OK;            // UnMount

  if (card_initialize() & STA_NOINIT)       // Check if card is ready or not
    return FR_NOT_READY;

  // Search FAT32 partition on the drive
  ///////////////////////////////////////////////////////////
  bsect = 0;
  fmt = check_fs(buf, bsect);         // Check sector 0 as an SFD format
  
  if( fmt == 1 ) {            // Not an FAT boot record, it may be FDISK format
    // Check the first partition in partition table
    if (card_readp(buf, bsect, MBR_Table, 16)) {    // 1st partition entry
      fmt = 3; // unable to read
    } else {
      if ( buf[4]==0x0B || buf[4]==0x0C ) {   // Check partition type for FAT32
        bsect = (uint32_t)(*(uint32_t*)(uint8_t*)&buf[8]);    // LBA Begin of partition
        fmt = check_fs(buf, bsect);   // Check the partition 
      }
      else fmt = 1; // not fat32
    }
  }
  
  if( fmt == 3 ) return FR_DISK_ERR;
  if( fmt > 0 ) return FR_NO_FILESYSTEM;        // No valid FAT32 patition is found
  ///////////////////////////////////////////////////////////
  
  // Initialize the file system object
  if (card_readp(buf, bsect, 0x0D, sizeof(buf))) return FR_DISK_ERR;
  
  fs->csize = buf[BPB_SecPerClus];        // Number of sectors per cluster

  fsize = (uint32_t)(*(uint32_t*)(uint8_t*)&buf[BPB_FATSz32]) * buf[BPB_NumFATs];
  rsvsect = (uint16_t)(*(uint16_t*)(uint8_t*)&buf[BPB_RsvdSecCnt]);
  
  // Maximum cluster# + 1. Number of clusters is max_clust - 2
  fs->max_clust = ((uint32_t)(*(uint32_t*)(uint8_t*)&buf[BPB_TotSec32]) - rsvsect - fsize) / fs->csize + 2;
  
  fs->dirbase = (uint32_t)(*(uint32_t*)(uint8_t*)&buf[BPB_RootClus]);     // Root directory first cluster

  fs->fatbase = bsect + rsvsect;          // FAT begin LBA
  fs->database = fs->fatbase + fsize;       // Cluster begin LBA

  FatFs = fs;

  return FR_OK;
}


/// Open or Create a File
uint8_t pf_open (const char *path)
{
    FRESULT res;
    DIR dj;
    uint8_t sp[12];
    FATFS *fs = FatFs;
    uint8_t *dir;

    if (!fs) return FR_NOT_ENABLED;       // Check if file system is mounted

    dir = fs->buf;
    
    dj.fn = sp;
    res = follow_path(&dj, path);  /* Follow the file path */
    if (res != FR_OK) return res;         // File not found

    if (!dir[0] || (dir[DIR_Attr] & AM_DIR))      // It is a directory
        return FR_NO_FILE;

    fs->org_clust = get_clust(dir);    /* File start cluster */
  
    fs->fsize = (uint32_t)(*(uint32_t*)(uint8_t*)&dir[DIR_FileSize]);       // File size
    fs->fptr = 0;                         // File pointer

    return FR_OK;
}


/// Read File
uint16_t pf_read (
  void* dest,       // Pointer to the destination object
  uint16_t btr      // Number of bytes to read (bit15:destination)
)
{
  uint16_t bread = 0, rcnt;
  uint32_t clst;
  uint32_t sect, remain;
  uint8_t *rbuff = (uint8_t*)dest;
  FATFS *fs = FatFs;

  if (!fs) return 0;            // Check file system is mounted

  remain = fs->fsize - fs->fptr;
  if (btr > remain) btr = (uint16_t)remain;       // Truncate btr by remaining bytes
  
  for (;btr;rbuff += rcnt, fs->fptr += rcnt, bread += rcnt, btr -= rcnt)
  { // Repeat until all data transferred
    if ((fs->fptr & 511) == 0)
    { // On the sector boundary?
      if (((fs->fptr >> 9) & (fs->csize - 1)) == 0)
      { // On the cluster boundary?
        clst = (fs->fptr == 0) ?    // On the top of the file?
          fs->org_clust : get_fat(fs->curr_clust);
        if (clst <= 1) return 0;
        fs->curr_clust = clst;      // Update current cluster
        fs->csect = 0;        // Reset sector offset in the cluster
      }
      sect = clust2sect(fs->curr_clust);    // Get current sector
      if (!sect) {
        return 0;
      }
      sect += fs->csect;
      fs->dsect = sect;
      fs->csect++;          // Next sector address in the cluster
    }
    rcnt = 512 - ((uint16_t)fs->fptr & 511);      // Get partial sector data from sector buffer
    if (rcnt > btr) rcnt = btr;
    if (card_readp(rbuff, fs->dsect, (uint16_t)(fs->fptr & 511), rcnt))
      return 0;
  }

  return bread;
}


/// Seek File Pointer
FRESULT pf_lseek (
  uint32_t ofs   /* File pointer from top of file */
)
{
  uint32_t clst;
  uint32_t bcs, sect, ifptr;
  FATFS *fs = FatFs;


  //if (!fs) return FR_NOT_ENABLED;   /* Check file system */
  //if (!(fs->flag & FA_OPENED))    /* Check if opened */
      //return FR_NOT_OPENED;

  if (ofs > fs->fsize) ofs = fs->fsize; /* Clip offset with the file size */
  ifptr = fs->fptr;
  fs->fptr = 0;
  if (ofs > 0)
  {
    bcs = (uint32_t)fs->csize * 512; /* Cluster size (byte) */
    if (ifptr > 0 &&
      (ofs - 1) / bcs >= (ifptr - 1) / bcs) { /* When seek to same or following cluster, */
      fs->fptr = (ifptr - 1) & ~(bcs - 1);  /* start from the current cluster */
      ofs -= fs->fptr;
      clst = fs->curr_clust;
    } else {              /* When seek to back cluster, */
      clst = fs->org_clust;     /* start from the first cluster */
      fs->curr_clust = clst;
    }
    while (ofs > bcs) {       /* Cluster following loop */
      clst = get_fat(clst);   /* Follow cluster chain */
      if (clst <= 1 || clst >= fs->max_clust) return FR_DISK_ERR;
      fs->curr_clust = clst;
      fs->fptr += bcs;
      ofs -= bcs;
    }
    fs->fptr += ofs;
    sect = clust2sect(clst);    /* Current sector */
    if (!sect) return FR_DISK_ERR;
    fs->dsect = sect + ((fs->fptr >> 9) & (fs->csize - 1));
  }
  else
  {
    fs->curr_clust = fs->org_clust;
    fs->fptr = 0;
    fs->csect = clust2sect(fs->curr_clust);    /* Current sector */
    fs->dsect = fs->csect + ((fs->fptr >> 9) & (fs->csize - 1));
  }

  return FR_OK;
}


/// Open Directory - Create a Directroy Object
FRESULT pf_opendir (
    DIR *dj,      // Pointer to directory object to create
    const char *path    // Pointer to the directory path
)
{
    FRESULT res;
    uint8_t sp[12];
    FATFS *fs = FatFs;
    uint8_t *dir;
    dj->index = 0;

    if (!fs) {              // Check file system is mounted
        res = FR_NOT_ENABLED;
    } else {
        dj->fn = sp;
        dir = FatFs->buf;
        res = follow_path(dj, path);    /* Follow the path to the directory */
        if (res == FR_OK) {         // File found
            if (dir[0]) {         // It is not empty dir
                if (dir[DIR_Attr] & AM_DIR)   // The object is a directory
                    dj->sclust = get_clust(dir);
                 else          // The object is not a directory
                    res = FR_NO_FILE;
            }
            if (res == FR_OK) res = dir_rewind(dj);   // Rewind dir
        }
    }

    return res;
}


/// Read current Directory Entry
FRESULT pf_readdir (
    DIR *dj,          // Pointer to the open directory object
    FILINFO *fno,      // Pointer to file information to return
    uint8_t move_direction
)
{
    FRESULT res;
    uint8_t sp[12];
    FATFS *fs = FatFs;
    uint8_t *dir;


    if (!fs)              // Check file system is mounted
        res = FR_NOT_ENABLED;
    else
    {
        dir = fs->buf;
        dj->fn = sp;
        if (!fno)
            res = dir_rewind(dj);
        else
        {
            res = dir_read(dj, move_direction);
            if (res == FR_NO_FILE)
            {
                dj->sect = 0;
                res = FR_OK;
            }
            if (res == FR_OK)       // A valid entry is found
                get_fileinfo(dj, fno);  // Get the object information
        }
    }

    return res;
}
