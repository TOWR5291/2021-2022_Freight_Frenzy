package club.towr5291.libraries;

import android.graphics.Bitmap;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import club.towr5291.functions.FileLogger;

public class ImageCaptureOCV {

    private VuforiaLocalizer libraryVuforia;
    private Mat currentMat;
    private TOWRDashBoard dash;
    private FileLogger logger;
    private Image rgbImage;
//    private VuforiaLocalizer vuforia = null;


    public interface OnImageCapture {
        void OnImageCaptureVoid(Mat mat);
    }

    public ImageCaptureOCV() {
        //Nothing in here yet
    }

    public void initImageCaptureOCV(LibraryVuforia libraryVuforia, TOWRDashBoard dashBoard, FileLogger fileLogger) {
        this.libraryVuforia = libraryVuforia.getVuforia();
        this.currentMat = new Mat();
        this.dash = dashBoard;
        this.logger = fileLogger;
        this.rgbImage = null;
    }

    public void initImageCaptureOCV(VuforiaLocalizer vuforiaLocalizer, TOWRDashBoard dashBoard, FileLogger fileLogger) {

        this.libraryVuforia = vuforiaLocalizer;
//        this.currentMat = new Mat();
        this.dash = dashBoard;
        this.logger = fileLogger;
        this.rgbImage = null;
    }

    public void takeImage(OnImageCapture onImageCapture) {

        try {
            VuforiaLocalizer.CloseableFrame frame = this.libraryVuforia.getFrameQueue().take();
            dash.displayPrintf(0, "Check Point 1");

            Thread.sleep(1000);
            ///  this.libraryVuforiaRoverRuckus.getVuforia().getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            //VuforiaLocalizer.CloseableFrame frame = this.
            ///        @Override
            ///         public void accept(Frame frame ) {
            // VuforiaLocalizer.CloseableFrame frame = this.libraryVuforiaRoverRuckus.getVuforia().getFrameOnce(Continuation.create(ThreadPool.getDefault(),
//            vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            //  @Override
            // public void accept(Frame frame) {
            //VuforiaLocalizer.CloseableFrame frame = this.libraryVuforiaRoverRuckus.getVuforia().getFrameQueue().take(); //takes the frame at the head of the queue
            long numImages = frame.getNumImages();
            dash.displayPrintf(0, "Check Point 2");

            Thread.sleep(1000);
            if (frame == null) {
                logger.writeEvent(3, "VISION", "Yep Null " + numImages);
            } else {
                logger.writeEvent(3, "VISION", "Nope not Null " + numImages);
            }
            logger.writeEvent(3, "VISION", "Number of Images " + numImages);
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    logger.writeEvent(3, "VISION", "Did this happen? " + numImages);
                    this.rgbImage = frame.getImage(i);
                    break;
                } else {
                    logger.writeEvent(3, "VISION", "This did not happen " + numImages);
                }
            }
            dash.displayPrintf(0, "Check Point 3");

            Thread.sleep(1000);
            logger.writeEvent(3, "VISION", "Right Before Frame to BM" + numImages);
            //Bitmap bitmap = libraryVuforiaRoverRuckus.getVuforia().convertFrameToBitmap(frame);
            //              Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
            /*rgb is now the Image object that we’ve used in the video*/
            Bitmap bitmap = Bitmap.createBitmap(this.rgbImage.getWidth(), this.rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(this.rgbImage.getPixels());
            logger.writeEvent(3, "VISION", "Right After Frame to BM" + numImages);
            //put the image into a MAT for OpenCV
            //if (bitmap != null)
            dash.displayPrintf(0, "Check Point 4");
            Thread.sleep(1000);
            currentMat = new Mat(this.rgbImage.getWidth(), this.rgbImage.getHeight(), CvType.CV_8UC4);
            //}
            //else {
            //    logger.writeEvent(3, "VISION", "Yeah...was null " + numImages);
            //}
            int bmwidth = bitmap.getWidth();
            int bmheight = bitmap.getHeight();
            logger.writeEvent(3, "VISION", "Bitmaps Width is " + bmwidth);
            logger.writeEvent(3, "VISION", "Bitmaps Height is " + bmheight);
            int rgbwidth = rgbImage.getWidth();
            int rgbheight = rgbImage.getHeight();
            logger.writeEvent(3, "VISION", "RGBs Width is " + rgbwidth);
            logger.writeEvent(3, "VISION", "RGBs Height is " + rgbheight);
            //int allzeros = countNonZero(currentMat);
            //logger.writeEvent(3, "VISION", "Prior non-zero values in mat " + allzeros);
            Utils.bitmapToMat(bitmap, this.currentMat);
            frame.close();
        } catch (InterruptedException e) {
            dash.displayPrintf(1, "VUFORIA --- ERROR ERROR ERROR");
            dash.displayPrintf(2, "VUFORIA --- ERROR ERROR ERROR");
        }


        onImageCapture.OnImageCaptureVoid(currentMat);
    }
}