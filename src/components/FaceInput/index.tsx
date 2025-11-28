import React, { useRef, useState, ReactNode } from 'react';
import styles from './styles.module.css';

export default function FaceInput(): ReactNode {
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [capturedImage, setCapturedImage] = useState<string | null>(null);
  const [isStreaming, setIsStreaming] = useState<boolean>(false);

  const startWebcam = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true });
      if (videoRef.current) {
        videoRef.current.srcObject = stream;
        setIsStreaming(true);
      }
    } catch (err) {
      console.error("Error accessing webcam: ", err);
    }
  };

  const captureFrame = () => {
    if (videoRef.current && canvasRef.current) {
      const context = canvasRef.current.getContext('2d');
      if (context) {
        const { videoWidth, videoHeight } = videoRef.current;
        canvasRef.current.width = videoWidth;
        canvasRef.current.height = videoHeight;
        context.drawImage(videoRef.current, 0, 0, videoWidth, videoHeight);
        const imageUrl = canvasRef.current.toDataURL('image/png');
        setCapturedImage(imageUrl);
        stopWebcam();
      }
    }
  };

  const stopWebcam = () => {
    if (videoRef.current && videoRef.current.srcObject) {
      const stream = videoRef.current.srcObject as MediaStream;
      stream.getTracks().forEach(track => track.stop());
      videoRef.current.srcObject = null;
      setIsStreaming(false);
    }
  };

  const resetCapture = () => {
    setCapturedImage(null);
    startWebcam();
  };

  return (
    <div className={styles.faceInputContainer}>
      <h3 className={styles.title}>User Face Input</h3>
      <div className={styles.videoContainer}>
        {!capturedImage && (
          <video ref={videoRef} autoPlay playsInline className={styles.video} />
        )}
        {capturedImage && (
          <img src={capturedImage} alt="Captured frame" className={styles.capturedImage} />
        )}
        <canvas ref={canvasRef} style={{ display: 'none' }} />
      </div>
      <div className={styles.buttonContainer}>
        {!isStreaming && !capturedImage && (
          <button onClick={startWebcam} className="button button--primary">Start Webcam</button>
        )}
        {isStreaming && (
          <button onClick={captureFrame} className="button button--success">Capture</button>
        )}
        {isStreaming && (
          <button onClick={stopWebcam} className="button button--danger">Stop Webcam</button>
        )}
        {capturedImage && (
          <button onClick={resetCapture} className="button button--secondary">Retake</button>
        )}
      </div>
    </div>
  );
}
