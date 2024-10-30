// Define structure to hold location information
struct Location {
    double latitude;
    double longitude;
    int openingHours[7][4]; // Opening hours for each day mon to sun { hours start, minutes start, hours end, minutes end }
};

// Define an array to hold location data with opening hours
const Location locations[] = {
    { 52.3676368, 9.7545293,
        {
            {8, 0, 0, 0},
            {8, 0, 0, 0},
            {8, 0, 0, 0},
            {8, 0, 0, 0},
            {8, 0, 1, 0},
            {8, 0, 1, 0},
            {9, 0, 0, 0},
        }
    },
    { 52.3908996, 9.72050896905512, 
        {
            {5, 0, 4, 0},
            {5, 0, 4, 0},
            {5, 0, 4, 0},
            {5, 0, 4, 0},
            {5, 0, 4, 0},
            {5, 0, 4, 0},
            {5, 0, 4, 0},
        }
    }
};
