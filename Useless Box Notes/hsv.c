#include <stdio.h>
#include <math.h>

// Function to convert HSV to RGB
void HSV_to_RGB(float H, float S, float V, int *R, int *G, int *B) {
    float C = V * S;
    float X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
    float m = V - C;

    float r, g, b;
    if (H >= 0 && H < 60) {
        r = C, g = X, b = 0;
    } else if (H >= 60 && H < 120) {
        r = X, g = C, b = 0;
    } else if (H >= 120 && H < 180) {
        r = 0, g = C, b = X;
    } else if (H >= 180 && H < 240) {
        r = 0, g = X, b = C;
    } else if (H >= 240 && H < 300) {
        r = X, g = 0, b = C;
    } else {
        r = C, g = 0, b = X;
    }

    // Convert to 8-bit RGB
    *R = (int)((r + m) * 255);
    *G = (int)((g + m) * 255);
    *B = (int)((b + m) * 255);
}

// Example usage
int main(){
    int R, G, B;
    int K = 4096;
    int H;
    // Input from potentiometer could be [0,255], or [0,4096], etc. Call largest K
    // H must be in [0, 360]
    // Convert i ([0,K]) to H ([0,360])
    // H = i * 360 / K
    for(int i = 0; i <= K; i++){
        H = i * 360 / K;
        HSV_to_RGB(H, 1, 1, &R, &G, &B);
        printf("R: %d, G: %d, B: %d\n", R, G, B);
    }
    return 0;
}


