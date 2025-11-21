import numpy as np
import pandas as pd
from scipy import interpolate
from scipy import stats
from scipy.odr import ODR, Model, RealData
import warnings
warnings.filterwarnings('ignore', category=RuntimeWarning)


def seebeck_calc(seebeckData,method):

    # data: pandas DataFrame with columns ['time','T_left','T_right','V']
    df = pd.DataFrame(seebeckData, names=["Temperature_Difference_C", "Voltage_mV", "Temp1_C", "Temp2_C", "Timestamp"])
    

    # 1. compute deltaT
    df['dT'] = df['Temp1_C'] - df['Temp2_C']   # or T_hot - T_cold as your sign convention

    # 2. segment into cycles by sign of derivative (simple method)
    df['d_dT_dt'] = df['dT'].diff() / df['Timestamp'].diff()
    # find zeros or big sign changes to cut cycles; here we group by runs of monotonic dT sign
    df['direction'] = np.sign(df['d_dT_dt'].fillna(0))
    df['cycle_id'] = (df['direction'] != df['direction'].shift()).cumsum()

    # OPTIONAL: if you know exact cycle boundaries, assign cycle_id accordingly

    # 3. build common dT grid
    dT_min, dT_max = df['dT'].min(), df['dT'].max()
    dT_grid = np.arange(dT_min, dT_max + 1e-6, 1.0)  # 0.2 °C spacing, choose appropriately

    # 4. interpolate each cycle onto grid
    cycles = df['cycle_id'].unique()
    V_grid_all = np.full((len(cycles), len(dT_grid)), np.nan)

    for i, c in enumerate(cycles):
        sub = df[df['cycle_id']==c].dropna(subset=['dT','Voltage_mV'])
        if len(sub) < 5:
            continue
        # require monotonic dT for safe interpolation; if not monotonic, sort by dT
        sub_sorted = sub.sort_values('dT')
        # avoid duplicate dT values for interp
        dT_vals, V_vals = np.unique(sub_sorted['dT'].values, return_index=False), None
        # simpler: use interp1d with bounds_error=False
        f = interpolate.interp1d(sub_sorted['dT'].values, sub_sorted['Voltage_mV'].values,
                                kind='linear', bounds_error=False, fill_value=np.nan)
        V_grid_all[i,:] = f(dT_grid)

    # 5. average across cycles
    valid_rows = ~np.all(np.isnan(V_grid_all), axis=1)
    if np.any(valid_rows):
        mean_V = np.nanmean(V_grid_all[valid_rows, :], axis=0)
        std_V = np.nanstd(V_grid_all[valid_rows, :], axis=0)
    else:
        raise ValueError("No valid data found for averaging.")

        #mean_V = np.nanmean(V_grid_all, axis=0)
        #std_V = np.nanstd(V_grid_all, axis=0)

    # 6. linear fit to averaged curve (use only grid points with enough samples)
    valid = ~np.isnan(mean_V)
    x = dT_grid[valid]
    y = mean_V[valid]
    w = 1.0 / (std_V[valid] + 1e-12)  # simple weights; tune as needed

    # Option A: simple weighted least squares
    slope, intercept, r_val, p_val, stderr = stats.linregress(x, y)  # unweighted simple fit

    # Option B: weighted polyfit
    coeffs = np.polyfit(x, y, 1, w=w)
    slope_w, intercept_w = coeffs[0], coeffs[1]

    # Option C: orthogonal distance regression (if dT also uncertain)
    def linear_func(B, x):
        return B[0]*x + B[1]
    linear_model = Model(linear_func)
    data_for_odr = RealData(x, y)   # could include sx, sy if known
    odr = ODR(data_for_odr, linear_model, beta0=[slope_w, intercept_w])
    out = odr.run()
    slope_odr, intercept_odr = out.beta


    #print("Slope (polyfit weighted) = {:.6f} mV/°C".format(slope_w))
    #print("Slope (ODR) = {:.6f} mV/°C".format(slope_odr))

    seebeck_pfw = slope_w
    seebeck_odr = slope_odr


    # 7. bootstrap for error estimate (resample cycles)
    nboot = 500
    boot_slopes = []
    rng = np.random.default_rng(42)
    for _ in range(nboot):
        idx = rng.integers(0, len(cycles), len(cycles))

        V_boot = np.nanmean(V_grid_all[idx,:], axis=0)
        valid_b = ~np.isnan(V_boot)
        if np.sum(valid_b) < 3:
            continue
        coeffs_b = np.polyfit(dT_grid[valid_b], V_boot[valid_b], 1)
        boot_slopes.append(coeffs_b[0])
    boot_slopes = np.array(boot_slopes)
    #print("Bootstrap slope mean = {:.6f}, stdev = {:.6f}".format(boot_slopes.mean(), boot_slopes.std()))

    if method == "odr":
        seebeckCalc = seebeck_odr
        seebeckCalc_STDEV = boot_slopes.std()
    else:
        seebeckCalc = seebeck_pfw
        seebeckCalc_STDEV = boot_slopes.std()


    return seebeckCalc, seebeckCalc_STDEV