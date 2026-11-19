using System;
using System.Runtime.InteropServices;
[ComImport,Guid("BCDE0395-E52F-467C-8E3D-C4579291692E")] class Enumerator {}
[ComImport,Guid("A95664D2-9614-4F35-A746-DE8DB63617E6"),InterfaceType(ComInterfaceType.InterfaceIsIUnknown)] interface IEnum {
 int EnumAudioEndpoints(int flow,int mask,out IntPtr devices);
 int GetDefaultAudioEndpoint(int flow,int role,out IDevice device);
}
[ComImport,Guid("D666063F-1587-4E43-81F1-B948E807363F"),InterfaceType(ComInterfaceType.InterfaceIsIUnknown)] interface IDevice {
 int Activate(ref Guid id,int context,IntPtr args,[MarshalAs(UnmanagedType.IUnknown)] out object obj);
 int OpenPropertyStore(int access,out IntPtr store);
 int GetId([MarshalAs(UnmanagedType.LPWStr)] out string id);
 int GetState(out int state);
}
[ComImport,Guid("5CDF2C82-841E-4546-9722-0CF74078229A"),InterfaceType(ComInterfaceType.InterfaceIsIUnknown)] interface IVolume {
 int RegisterControlChangeNotify(IntPtr p); int UnregisterControlChangeNotify(IntPtr p); int GetChannelCount(out uint count);
 int SetMasterVolumeLevel(float level,Guid context); int SetMasterVolumeLevelScalar(float level,Guid context);
 int GetMasterVolumeLevel(out float level); int GetMasterVolumeLevelScalar(out float level);
 int SetChannelVolumeLevel(uint channel,float level,Guid context); int SetChannelVolumeLevelScalar(uint channel,float level,Guid context);
 int GetChannelVolumeLevel(uint channel,out float level); int GetChannelVolumeLevelScalar(uint channel,out float level);
 int SetMute([MarshalAs(UnmanagedType.Bool)] bool mute,Guid context); int GetMute([MarshalAs(UnmanagedType.Bool)] out bool mute);
}
public class AudioCheck {
 public static void EnableQuietly() { IDevice device; ((IEnum)new Enumerator()).GetDefaultAudioEndpoint(0,1,out device); object obj; Guid iid=typeof(IVolume).GUID; device.Activate(ref iid,23,IntPtr.Zero,out obj); ((IVolume)obj).SetMasterVolumeLevelScalar(.2f,Guid.Empty); ((IVolume)obj).SetMute(false,Guid.Empty); } public static string Read() { IDevice device; ((IEnum)new Enumerator()).GetDefaultAudioEndpoint(0,1,out device); string id; device.GetId(out id); object obj; Guid iid=typeof(IVolume).GUID; device.Activate(ref iid,23,IntPtr.Zero,out obj); float level; bool mute; ((IVolume)obj).GetMasterVolumeLevelScalar(out level); ((IVolume)obj).GetMute(out mute); return "Endpoint="+id+" Volume="+level+" Mute="+mute; }
}

