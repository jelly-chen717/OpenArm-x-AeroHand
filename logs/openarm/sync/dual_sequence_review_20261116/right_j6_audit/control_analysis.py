"""Offline reconstruction of right-J6 control behavior; never imports a CAN driver."""
import hashlib
import json
import math
import statistics
from pathlib import Path

OUT = Path(__file__).resolve().parent
RUN = OUT.parent/'execution_20261116_retry1'
LOG = RUN/'execution.jsonl'
ROOT = OUT.parents[4]
rows = [json.loads(line) for line in LOG.read_text(encoding='utf-8').splitlines() if line.strip()]
samples = [row for row in rows if row['type']=='sample']
session = next(row for row in rows if row['type']=='session')
keys = [tuple(key) for key in session['keys']]
index = keys.index(('right',6))
request = json.loads((RUN/'request.authorized.json').read_text(encoding='utf-8'))
settings = next(row for row in rows if row['type']=='joint_settings')
cfg = next(row for row in settings['joints'] if (row['side'],row['motor_id'])==('right',6))
mapping_file = ROOT/'configs/openarm_joint_mapping.json'
mapping = json.loads(mapping_file.read_text(encoding='utf-8'))
right6_mapping = next(row for row in mapping['joints'] if (row['side'],row['motor_id'])==('right',6))
assert mapping['version']==session['mapping_version']
assert right6_mapping['hardware_mapping']==dict(direction=1,scale=1.,p_ref=0.,q_ref=0.)
parameter_file = OUT/'hardware_parameters.json'
hardware = json.loads(parameter_file.read_text(encoding='utf-8'))
hardware_j6 = {row['side']:row for row in hardware['rows'] if row['motor_id']==6}
parameter_values = {side:{name:record['value'] for name,record in row['parameters'].items()}
                    for side,row in hardware_j6.items()}
parameter_differences = {name:dict(left=value,right=parameter_values['right'][name])
                         for name,value in parameter_values['left'].items() if value!=parameter_values['right'][name]}


def quantize(value, lo, hi, bits):
    code = int((value-lo)/(hi-lo)*((1<<bits)-1))
    return code, code/((1<<bits)-1)*(hi-lo)+lo


def encoded(command):
    # Mirrors apps/openarm_driver.py encode_mit exactly: right J6 has a +1,
    # 1:1 hardware map, DM4310 position/velocity/torque ranges +/-12.5/30/10.
    specs = ((command['target'][index],-12.5,12.5,16),
             (command['target_velocity'][index],-30.,30.,12),
             (cfg['kp'],0.,500.,12),(cfg['kd'],0.,5.,12),(0.,-10.,10.,12))
    pairs = [quantize(*spec) for spec in specs]
    p,v,k,d,t = [pair[0] for pair in pairs]
    packet = bytes((p>>8,p&255,v>>4,((v&15)<<4)|(k>>8),k&255,
                    d>>4,((d&15)<<4)|(t>>8),t&255))
    return dict(zip(('p','v','kp','kd','tau_ff'),[pair[1] for pair in pairs])),packet.hex()


def motor(row):
    return next(m for m in row['motor_feedback'] if (m['side'],m['motor_id'])==('right',6))


def predict(command, measurement):
    values,_ = encoded(command)
    measured = motor(measurement)
    pterm = values['kp']*(values['p']-measured['position_rad'])
    dterm = values['kd']*(values['v']-measured['velocity_rad_s'])
    total = pterm+dterm+values['tau_ff']
    return dict(p_nm=pterm,d_nm=dterm,ff_nm=values['tau_ff'],total_nm=total,
                feedback_tau_nm=measured['torque_nm'],residual_nm=measured['torque_nm']-total)


def metric(values):
    return dict(n=len(values),min=min(values),max=max(values),mean=statistics.mean(values),
                max_abs=max(map(abs,values)))


def residual_stats(values):
    return dict(n=len(values),mean_nm=statistics.mean(values),mae_nm=statistics.mean(map(abs,values)),
                rms_nm=math.sqrt(statistics.mean(v*v for v in values)),max_abs_nm=max(map(abs,values)))


segments=[]
for stage in ('J6','J5','J4','J3','J2'):
    for phase in ('moving','settling'):
        selected=[(i,row) for i,row in enumerate(samples) if row['stage']==stage and row['phase']==phase]
        if not selected:
            continue
        subset=[row for i,row in selected]
        pds=[predict(row,row) for row in subset]
        lags=[]
        for lag in (0,1,2):
            pairs=[(samples[i-lag],row) for i,row in selected if i>=lag]
            differences=[predict(a,b)['residual_nm'] for a,b in pairs]
            lags.append(dict(command_lag_cycles=lag,
                actual_command_age_s=metric([b['t']-a['t'] for a,b in pairs]),
                **residual_stats(differences)))
        segments.append(dict(stage=stage,phase=phase,start_s=subset[0]['t'],end_s=subset[-1]['t'],
            observed_span_s=subset[-1]['t']-subset[0]['t'],sample_count=len(subset),
            all_commanded_joint_velocities_zero=all(all(v==0 for v in row['target_velocity']) for row in subset),
            right_j6_command_position_constant=len({row['target'][index] for row in subset})==1,
            error_deg=metric([math.degrees(row['error'][index]) for row in subset]),
            measured_velocity_rad_s=metric([row['measured_velocity'][index] for row in subset]),
            measured_position_span_deg=math.degrees(max(row['measured'][index] for row in subset)-min(row['measured'][index] for row in subset)),
            torque_nm=metric([motor(row)['torque_nm'] for row in subset]),
            pd_same_frame=residual_stats([pd['residual_nm'] for pd in pds]),
            command_lag_sensitivity=lags,
            max_dispatch_cycle_s=max(row['dispatch']['cycle_s'] for row in subset),
            same_frame_basis='position, velocity and torque decoded together from one feedback frame after this cycle sent the MIT batch'))

j2=[row for row in samples if row['stage']=='J2']
trajectory=[]
for row in j2:
    pd=predict(row,row)
    trajectory.append(dict(t=row['t'],error_deg=math.degrees(row['error'][index]),
        measured_velocity_rad_s=row['measured_velocity'][index],
        torque_feedback_nm=motor(row)['torque_nm'],pd=pd,
        payload_reconstructed_hex=encoded(row)[1]))
last=samples[-1]
last_values,last_packet=encoded(last)
done=[row for row in rows if row['type']=='stage_complete']
firmware_states=sorted({motor(row)['state'] for row in samples})
recorded_faults=[row for row in rows if row['type'] in ('fault','hold_tolerance_exceeded','stage_timeout')]
stop=next(row for row in rows if row.get('state')=='STOPPED')
after=next(row for row in rows if row['type']=='after')
after_joint=next(m for report in after['reports'] if report['side']=='right' for m in report['motors'] if m['motor_id']==6)
report=dict(source=str(LOG),source_sha256=hashlib.sha256(LOG.read_bytes()).hexdigest(),
    analysis_scope='Offline retry1 log and source-code reconstruction only; no CAN, enable, commands, retries or control edits.',
    main_conclusion='Right J6 passed the recorded near-static hold intervals before J2. Failure occurred while J2 was still moving; static holding at the J2 endpoint has not been tested.',
    reached_stages=[row['stage'] for row in rows if row['type']=='stage_start'],
    completed_stages=[dict(stage=row['stage'],t=row['t'],continuous_dwell_s=row['dwell_s']) for row in done],
    segments=segments,j2_motion_samples=trajectory,
    encoding=dict(source='apps/openarm_driver.py:174-189',motor='DM4310',
        position_range_rad=[-12.5,12.5],velocity_range_rad_s=[-30,30],torque_range_nm=[-10,10],
        requested_kp=cfg['kp'],requested_kd=cfg['kd'],requested_tau_ff_nm=0.,
        reconstructed_final_command=last_values,reconstructed_final_payload_hex=last_packet,
        velocity_lsb_rad_s=60/4095,position_lsb_deg=math.degrees(25/65535),torque_lsb_nm=20/4095,
        note='The logged commands/settings and current encoder implementation reconstruct bytes, but actual sent payload bytes were not separately logged. Truncation means nominal v=0 and tau_ff=0 decode slightly negative.'),
    final_pd_check=predict(last,last),
    static_gain_threshold_compatibility=dict(
        assumption='Ideal stationary MIT outer position PD, zero commanded velocity and zero feedforward, no position integral; ignore small packet quantization bias for this compatibility calculation.',
        holding_position_threshold_deg=2.,requested_kp=cfg['kp'],
        proportional_torque_at_2deg_nm=cfg['kp']*math.radians(2),
        quantized_kp_proportional_torque_at_2deg_nm=last_values['kp']*math.radians(2),
        examples=[dict(hypothetical_confirmed_static_load_nm=load,
            steady_error_at_kp60_deg=math.degrees(load/cfg['kp']),
            theoretical_min_effective_kp_for_2deg=load/math.radians(2)) for load in (2.4,3.)],
        conclusion='With Kp 60, a hypothetical static load above about 2.094 Nm requires more than 2 degrees of proportional position error. Thus the host 2-degree position guard can trip before the 3 Nm torque-feedback guard even when the motor has torque headroom.',
        qualification='The measured 2.396 Nm occurred during J2 motion. It is not a measured static load. Theoretical Kp lower bounds are parameter-compatibility calculations, not approved gain changes or validated stable gains.',
        internal_loop_note='Stored ASR/APR registers do not add an externally commanded position integral or gravity feedforward term to this MIT request; inner motor loops are outside this simple outer-PD reconstruction.'),
    read_only_parameter_audit=dict(source=str(parameter_file),
        sha256=hashlib.sha256(parameter_file.read_bytes()).hexdigest(),time_unix=hardware['time_unix'],
        no_enable_or_MIT_frames=hardware['no_enable_or_MIT_frames'],
        left_j6_parameters=parameter_values['left'],right_j6_parameters=parameter_values['right'],
        left_right_differences=parameter_differences,
        final_all_14_disabled=sum(len(report['motors']) for report in hardware['final'])==14 and
                              all(m['state']==0 for report in hardware['final'] for m in report['motors']),
        interpretation='Both J6 motors report MIT mode 1, Gr 10, protocol ranges 12.5/30/10, OC_Value approximately 0.8 and matching firmware/ASR/APR parameters; only Damp and Inertia differ slightly. Protocol TMAX is not the host rated torque threshold or a measured saturation flag. No unverified units are assigned to OC_Value.'),
    user_confirmed_conditions=dict(end_effector='Original gripper',
        external_interference='User did not observe cable pulling or structural contact; this is an observation, not proof of the cause.'),
    alignment=dict(
        feedback='p/v/tau are from the same received feedback CAN frame, avoiding mixing different sample indices.',
        command='The corresponding sample target was sent before feedback collection. Current command fits the J6 moving interval substantially better than a whole-cycle older command.',
        hold_sensitivity='During J2 the right-J6 command is constant. Shifting its command by 0, 1 or 2 control cycles gives identical PD residuals, so the failure-time PD agreement is not an artifact of choosing the latest target.',
        observed_cycle_max_s=max(row['dispatch']['cycle_s'] for row in samples),
        limits='No device-time clock or inner-loop torque filtering timestamps are logged. Host received_at is coarsely quantized, and sample t is recorded after all 14 feedback queries. Exact sub-cycle plant latency cannot be identified from these fields.'),
    protection=dict(firmware_states_in_samples=firmware_states,recorded_faults=recorded_faults,
        right_j6_peak_abs_velocity_rad_s=max(abs(motor(row)['velocity_rad_s']) for row in samples),
        right_j6_rated_speed_threshold_rad_s=cfg['max_speed'],
        right_j6_peak_abs_torque_nm=max(abs(motor(row)['torque_nm']) for row in samples),
        right_j6_rated_torque_threshold_nm=cfg['max_torque'],
        right_j6_torque_feedback_plateau_observed=False,
        trigger='Host non-active-joint position holding error > 2 degrees, not a reported firmware fault or speed/torque protection trip.',
        firmware_limiting_scope='The log and subsequent read-only parameter audit verify MIT mode/protocol ranges and matching left/right configuration. They do not expose runtime inner current-clamp saturation, bus voltage or power-supply current. Thus no observed limiting is different from proving all internal saturation absent.',
        final_state=stop,after_right_j6=after_joint),
    supported_interpretation=[
        'After its own J6 motion, right-J6 near-static error remained 0.581 to 0.712 degrees with 0.647 to 0.774 Nm torque magnitude.',
        'During the J5, J4 and J3 full-arm stationary command intervals, its reported position was constant, error 0.361991 degrees, speed magnitude at most 0.021978 rad/s and torque magnitude approximately 0.413 to 0.427 Nm.',
        'During J2 motion its error and torque rose together. The controller commanded restoring torque in the expected direction, with feedback close to the quantized MIT PD prediction.',
        'The right-J6 error and feedback torque grew during J2 motion, while finite PD stiffness permits a nonzero tracking offset. The PD reconstruction explains the controller response, not the physical source of the rising error/load. No particular gravity, inertia, friction, contact or cable mechanism is established by this trace.',
        'A sustained static failure at the final J2 angle is not established: J2 was still moving and neither its endpoint nor a subsequent steady-state dwell was reached.',
    ],
    missing_validation=[
        'Authorized observation of right J6 at a fixed J2 pose after all commanded and measured motion settles; retain the same test zero and distinguish transient from steady-state offset.',
        'If comparing a slower J2 move, compare error and torque against J2 angle as well as time, then examine the stationary endpoint separately. Lower acceleration need not reduce gravity-driven static torque.',
        'The original gripper is user-confirmed and the user did not observe cable pulling or contact. Further evidence is still needed to explain the discrepancy between low-load static platforms and the rising motion-time effort; neither cable interference nor derivative noise is an established cause.',
        'Device timestamps/current/voltage or verified drive-limit configuration would be needed for a firm claim about internal saturation; the present feedback alone does not provide those measurements.',
    ])
OUT.mkdir(parents=True,exist_ok=True)
(OUT/'control_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(conclusion=report['main_conclusion'],
    static_platforms=[{k:s[k] for k in ('stage','start_s','end_s','observed_span_s','error_deg','measured_velocity_rad_s','torque_nm','pd_same_frame')} for s in segments if s['phase']=='settling'],
    final_pd_check=report['final_pd_check'],encoding=report['encoding'],alignment=report['alignment'],
    output=str(OUT/'control_analysis.json')),ensure_ascii=False,indent=2))
