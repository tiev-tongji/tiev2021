
#include <roadrunner.h>
extern "C" {

}

#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <dirent.h>
#include <wordexp.h>
#include <libgen.h>

#include "global.h"

int dgc_carp_verbose = 0;

double dgc_get_time(void) {
    struct timeval tv;
    double t;

    if (gettimeofday(&tv, NULL) < 0)
        dgc_warning("dgc_get_time encountered error in gettimeofday : %s\n", strerror(errno));
    t = tv.tv_sec + tv.tv_usec / 1000000.0;
    return t;
}

static FILE *dgc_carp_output = NULL;

void print_error_string(char *header, const char *fname, char *message) {
    unsigned int i, j, k, header_len, message_len, count;
    char *dest_str;

    header_len = strlen(header);
    message_len = strlen(message);
    count = header_len;
    if (fname != NULL) count += strlen(fname) + 3;
    for (i = 0; i < message_len - 1; i++)
        if (message[i] == '\n') count += header_len + 1;
        else count++;
    count++;

    dest_str = (char*) calloc(count + 5, 1);
    dgc_test_alloc(dest_str);

    strcpy(dest_str, header);
    j = header_len;
    if (fname != NULL) {
        strcat(dest_str, fname);
        strcat(dest_str, " : ");
        j += strlen(fname) + 3;
    }
    for (i = 0; i < message_len; i++) {
        if (i == message_len - 1 || message[i] != '\n') {
            dest_str[j] = message[i];
            j++;
            dest_str[j] = '\0';
        }
        else {
            dest_str[j] = '\n';
            j++;
            for (k = 0; k < header_len; k++) {
                dest_str[j] = ' ';
                j++;
            }
            dest_str[j] = '\0';
        }
    }
    if (message[message_len - 1] != '\n') strcat(dest_str, "\n");

    if (dgc_carp_output == NULL) dgc_carp_output = stderr;

    fprintf(dgc_carp_output, "%s", dest_str);
    fflush(dgc_carp_output);
    free(dest_str);
}


void dgc_error(char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    //print_error_string("# ERROR: ", NULL, message);
}

void dgc_warning(char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    //print_error_string("# WARNING: ", NULL, message);
}

void dgc_info(char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    //print_error_string("# INFO: ", NULL, message);
}

void dgc_fatal_ferror(const char *fname, char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    //print_error_string("# ERROR: ", fname, message);
    exit(-1);
}

void dgc_ferror(const char *fname, char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    //print_error_string("# ERROR: ", fname, message);
}

void dgc_fwarning(const char *fname, char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    //print_error_string("# WARNING: ", fname, message);
}

void dgc_finfo(const char *fname, char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    //print_error_string("# INFO: ", fname, message);
}

void dgc_die(const char* fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    //print_error_string("# ERROR: ", NULL, message);
    exit(-1);
}

void dgc_verbose(const char *fmt, ...) {
    va_list args;

    if (!dgc_carp_verbose) return;

    if (dgc_carp_output == NULL) dgc_carp_output = stderr;

    va_start(args, fmt);
    vfprintf(dgc_carp_output, fmt, args);
    va_end(args);
    fflush(dgc_carp_output);
}

void dgc_carp_set_output(FILE *output) {
    dgc_carp_output = output;
}

void dgc_carp_set_verbose(int verbosity) {
    dgc_carp_verbose = verbosity;
}

int dgc_carp_get_verbose(void) {
    return dgc_carp_verbose;
}

double dgc_normalize_theta(double theta) {
    double multiplier;

    if (theta >= -TiEV_PI && theta < TiEV_PI) return theta;

    multiplier = floor(theta / (2 * TiEV_PI));
    theta = theta - multiplier * 2 * TiEV_PI;
    if (theta >= TiEV_PI) theta -= 2 * TiEV_PI;
    if (theta < -TiEV_PI) theta += 2 * TiEV_PI;

    return theta;
}

double dgc_average_angle(double theta1, double theta2) {
    double x, y;

    x = cos(theta1) + cos(theta2);
    y = sin(theta1) + sin(theta2);
    if (x == 0 && y == 0) {return 0;}
    return atan2(y, x);
}


int dgc_sign(double num) {
    if (num >= 0) return 1;
    return -1;
}

void dgc_rect_to_polar(double x, double y, double *r, double *theta) {
    *r = hypot(x, y);
    *theta = atan2(y, x);
}

unsigned int dgc_generate_random_seed(void) {
    FILE *random_fp;
    unsigned int seed;
    int ints;

    random_fp = fopen("/dev/random", "r");
    if (random_fp == NULL) {
        dgc_warning("Could not open /dev/random for reading: %s\n"
            "Using time ^ PID\n", strerror(errno));
        seed = time(NULL) ^ getpid();
        srandom(seed);
        return seed;
    }

    ints = fread(&seed, sizeof(int), 1, random_fp);
    if (ints != 1) {
        dgc_warning("Could not read an int from /dev/random: %s\n"
            "Using time ^ PID\n", strerror(errno));
        seed = time(NULL) ^ getpid();
        srandom(seed);
        return seed;
    }
    fclose(random_fp);
    srandom(seed);
    return seed;
}

unsigned int dgc_randomize(int *argc, char ***argv) {
    long long int user_seed;
    unsigned int seed;
    int bytes_to_move;
    int i;
    char *endptr;

    for (i = 0; i < *argc - 1; i++) {
        if (strcmp((*argv)[i], "--seed") == 0) {
            user_seed = strtoll((*argv)[i + 1], &endptr, 0);
            seed = (unsigned int) user_seed;
            if (endptr && *endptr != '\0') {
                dgc_warning("Bad random seed %s.\n", (*argv)[i + 1]);
                seed = dgc_generate_random_seed();
            }
            else if (seed != user_seed) {
                dgc_warning("Random seed too large: %s.\n", (*argv)[i + 1]);
                seed = dgc_generate_random_seed();
            }
            else {
                if (i < *argc - 2) {
                    bytes_to_move = (*argc - 2 - i) * sizeof(char *);
                    memmove((*argv) + i, (*argv) + i + 2, bytes_to_move);
                }
                (*argc) -= 2;
                srandom(seed);
            }
            return seed;
        }
    }
    seed = dgc_generate_random_seed();
    return seed;
}

void dgc_set_random_seed(unsigned int seed) {
    srand(seed);
}

/*
 From the rand(3) man page:

 In Numerical Recipes in C: The Art of Scientific Computing
 (William H. Press, Brian P.  Flannery, Saul A. Teukolsky,
 William T. Vetterling; New York: Cambridge University Press,
 1992 (2nd ed., p. 277)), the following comments are made:
 "If you want to generate a random integer  between  1  and  10,
 you  should always do it by using high-order bits, as in

 j=1+(int) (10.0*rand()/(RAND_MAX+1.0));

 and never by anything resembling

 j=1+(rand() % 10);

 (which uses lower-order bits)."
 */

int dgc_int_random(int max) {
    return (int) (max * (rand() / (RAND_MAX + 1.0)));
}

double dgc_uniform_random(double min, double max) {
    return min + (rand() / (double) RAND_MAX) * (max - min);
}

double dgc_gaussian_random(double mean, double std) {
    const double norm = 1.0 / (RAND_MAX + 1.0);
    double u = 1.0 - rand() * norm; /* can't let u == 0 */
    double v = rand() * norm;
    double z = sqrt(-2.0 * log(u)) * cos(2.0 * TiEV_PI * v);
    return mean + std * z;
}

int dgc_file_exists(char *filename) {
    FILE *fp;

    fp = fopen64(filename, "r");
    if (fp == NULL) return 0;
    else {
        fclose(fp);
        return 1;
    }
}

off64_t dgc_file_size(char *filename) {
    struct stat64 file_stat;

    stat64(filename, &file_stat);
    return file_stat.st_size;
}

char *dgc_file_extension(char *filename) {
    return strrchr(filename, '.');
}

int dgc_strcasecmp(const char *s1, const char *s2) {
    const unsigned char *p1 = (const unsigned char *) s1;
    const unsigned char *p2 = (const unsigned char *) s2;
    unsigned char c1, c2;

    if (p1 == p2) return 0;

    do {
        c1 = tolower(*p1++);
        c2 = tolower(*p2++);
        if (c1 == '\0') break;
    } while (c1 == c2);

    return c1 - c2;
}

int dgc_strncasecmp(const char *s1, const char *s2, size_t n) {
    const unsigned char *p1 = (const unsigned char *) s1;
    const unsigned char *p2 = (const unsigned char *) s2;
    unsigned char c1, c2;

    if (p1 == p2 || n == 0) return 0;

    do {
        c1 = tolower(*p1++);
        c2 = tolower(*p2++);
        if (c1 == '\0' || c1 != c2) return c1 - c2;
    } while (--n > 0);

    return c1 - c2;
}

char *dgc_new_stringv(const char *fmt, va_list args) {
    va_list ap;
    int n, size = 128;
    char *s;

    if (fmt == NULL) return NULL;

    s = (char *) calloc(size, sizeof(char));
    dgc_test_alloc(s);

    while (1) {
        va_copy(ap, args);
        n = vsnprintf(s, size, fmt, ap);
        va_end(ap);
        if (n < 0) {
            free(s);
            return NULL;
        }
        if (n >= 1 && n < size) return s;
        if (n >= 1) // glibc 2.1
        size = n + 1;
        else // glibc 2.0
        size *= 2;
        s = (char *) realloc(s, size * sizeof(char));
        dgc_test_alloc(s);
    }

    return NULL;
}

char *dgc_new_string(const char *fmt, ...) {
    va_list ap;
    char *s;

    va_start(ap, fmt);
    s = dgc_new_stringv(fmt, ap);
    va_end(ap);

    return s;
}

void dgc_print_version(void) {
    fprintf(stderr, "ROADRUNNER Software:  Version %d.%d.%d\n\n", DGC_MAJOR_VERSION, DGC_MINOR_VERSION, DGC_REVISION);
}

#define      MAX_NUM_ARGS            64
#define      MAX_LINE_LENGTH         1000


char *dgc_expand_filename(char *filename) {
    char *result = NULL, *output = NULL;
    wordexp_t p;

    wordexp(filename, &p, 0);
    if (p.we_wordc > 0) {
        result = (char *) realloc(result, strlen(p.we_wordv[0]) + 1);
        dgc_test_alloc(result);
        strcpy(result, p.we_wordv[0]);
        output = result;
    }
    p.we_offs = 0; // should avoid the non-aligned free under leopard (soka)
    wordfree(&p);
    return output;
}
