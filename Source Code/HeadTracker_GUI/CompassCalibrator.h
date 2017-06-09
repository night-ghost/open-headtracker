#include "AP_Math.h"
#include <math.h>
#include <stdint.h>

#define COMPASS_CAL_NUM_SPHERE_PARAMS 4
#define COMPASS_CAL_NUM_ELLIPSOID_PARAMS 9
#define COMPASS_CAL_NUM_SAMPLES 300

//RMS tolerance
#define COMPASS_CAL_DEFAULT_TOLERANCE 5.0f

enum compass_cal_status_t {
    COMPASS_CAL_NOT_STARTED=0,
    COMPASS_CAL_WAITING_TO_START=1,
    COMPASS_CAL_RUNNING_STEP_ONE=2,
    COMPASS_CAL_RUNNING_STEP_TWO=3,
    COMPASS_CAL_SUCCESS=4,
    COMPASS_CAL_FAILED=5
};

class CompassCalibrator {
public:
    typedef uint8_t completion_mask_t[10];

    CompassCalibrator();

    void start(bool retry, double delay, uint16_t offset_max);
    void clear();

    void update(bool &failure);
    void new_sample(const Vector3f &sample);

    bool check_for_timeout();

    bool running() const;

    void set_tolerance(double tolerance) { _tolerance = tolerance; }

    void get_calibration(Vector3f &offsets, Vector3f &diagonals, Vector3f &offdiagonals);

    double get_completion_percent() const;
    completion_mask_t& get_completion_mask();
    enum compass_cal_status_t get_status() const { return _status; }
    double get_fitness() const { return sqrt(_fitness); }
    uint8_t get_attempt() const { return _attempt; }

private:
    class param_t {
    public:
        double* get_sphere_params() {
            return &radius;
        }

        double* get_ellipsoid_params() {
            return &offset.x;
        }

        double radius;
        Vector3f offset;
        Vector3f diag;
        Vector3f offdiag;
    };

    class CompassSample {
    public:
        Vector3f get() const;
        void set(const Vector3f &in);
    private:
        int16_t x;
        int16_t y;
        int16_t z;
    };



    enum compass_cal_status_t _status;

    // timeout watchdog state
    uint32_t _last_sample_ms;

    // behavioral state
    double _delay_start_sec;
    uint32_t _start_time_ms;
    bool _retry;
    double _tolerance;
    uint8_t _attempt;
    uint16_t _offset_max;

    completion_mask_t _completion_mask;

    //fit state
    class param_t _params;
    uint16_t _fit_step;
    CompassSample *_sample_buffer;
    double _fitness; // mean squared residuals
    double _initial_fitness;
    double _sphere_lambda;
    double _ellipsoid_lambda;
    uint16_t _samples_collected;
    uint16_t _samples_thinned;

    bool set_status(compass_cal_status_t status);

    // returns true if sample should be added to buffer
    bool accept_sample(const Vector3f &sample);
    bool accept_sample(const CompassSample &sample);

    // returns true if fit is acceptable
    bool fit_acceptable();

    void reset_state();
    void initialize_fit();

    bool fitting() const;

    // thins out samples between step one and step two
    void thin_samples();

    double calc_residual(const Vector3f& sample, const param_t& params) const;
    double calc_mean_squared_residuals(const param_t& params) const;
    double calc_mean_squared_residuals() const;

    void calc_initial_offset();
    void calc_sphere_jacob(const Vector3f& sample, const param_t& params, double* ret) const;
    void run_sphere_fit();

    void calc_ellipsoid_jacob(const Vector3f& sample, const param_t& params, double* ret) const;
    void run_ellipsoid_fit();

    /**
     * Update #_completion_mask for the geodesic section of \p v. Corrections
     * are applied to \p v with #_params.
     *
     * @param v[in] A vector representing one calibration sample.
     */
    void update_completion_mask(const Vector3f& v);
    /**
     * Reset and update #_completion_mask with the current samples.
     */
    void update_completion_mask();
};
