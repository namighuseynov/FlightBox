using System.IO.MemoryMappedFiles;
using System.Threading;
using UnityEngine;

public class UnityMemoryStreamer : MonoBehaviour
{
    public RenderTexture sourceTexture;
    [Header("Chanel settings")]
    public string mmapName = "UnityBuffer_1";
    public string eventName = "UnityEvent_1";

    private RenderTexture tempFlipTexture;
    private Texture2D bufferTexture;
    private MemoryMappedFile mmf;
    private MemoryMappedViewAccessor accessor;
    private EventWaitHandle frameReadyEvent;

    private int width = 640;
    private int height = 480;

    void Start()
    {
        long bufferSize = width * height * 4;
        mmf = MemoryMappedFile.CreateOrOpen(mmapName, bufferSize);
        accessor = mmf.CreateViewAccessor();
        frameReadyEvent = new EventWaitHandle(false, EventResetMode.AutoReset, eventName);

        bufferTexture = new Texture2D(width, height, TextureFormat.BGRA32, false);
        tempFlipTexture = new RenderTexture(width, height, 0);
    }

    void Update()
    {
        if (sourceTexture == null) return;

        Graphics.Blit(sourceTexture, tempFlipTexture, new Vector2(1, -1), new Vector2(0, 1));
        RenderTexture.active = tempFlipTexture;
        bufferTexture.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        bufferTexture.Apply();

        byte[] rawData = bufferTexture.GetRawTextureData();
        accessor.WriteArray(0, rawData, 0, rawData.Length);

        frameReadyEvent.Set();
    }

    void OnDestroy()
    {
        accessor?.Dispose();
        mmf?.Dispose();
        frameReadyEvent?.Dispose();
        if (tempFlipTexture != null) tempFlipTexture.Release();
    }
}




