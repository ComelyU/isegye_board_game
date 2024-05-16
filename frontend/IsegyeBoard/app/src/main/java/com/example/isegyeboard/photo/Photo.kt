package com.example.isegyeboard.photo

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.BitmapFactory
import android.icu.text.SimpleDateFormat
import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageCapture
import androidx.camera.core.ImageCaptureException
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.navigation.findNavController
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import okhttp3.MediaType
import okhttp3.MultipartBody
import okhttp3.RequestBody
import java.io.File
import java.util.Locale
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

class Photo : Fragment() {

    private lateinit var outputDirectory: File
    private lateinit var cameraExecutor: ExecutorService
    private var imageCapture: ImageCapture? = null
    private lateinit var photoFile: File

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        return inflater.inflate(R.layout.fragment_photo, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        cameraExecutor = Executors.newSingleThreadExecutor()
        outputDirectory = getOutputDirectory()

        if (allPermissionsGranted()) {
            startCamera()
        } else {
            ActivityCompat.requestPermissions(
                requireActivity(), REQUIRED_PERMISSIONS, REQUEST_CODE_PERMISSIONS
            )
        }

        val previewText = view.findViewById<TextView>(R.id.previewText)
        val sendPhotoButton = view.findViewById<TextView>(R.id.sendPhotoButton)

        val captureButton = view.findViewById<TextView>(R.id.captureButton)
        captureButton.setOnClickListener {
            takePhoto()
            previewText.visibility = View.GONE
            sendPhotoButton.visibility = View.VISIBLE
        }

        sendPhotoButton.setOnClickListener{ sendPhoto() }

        val backButton = view.findViewById<ImageView>(R.id.photoBack)
        backButton.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }
    }

    private fun startCamera() {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(requireContext())
        cameraProviderFuture.addListener({
            val cameraProvider = cameraProviderFuture.get()
            val cameraView = view?.findViewById<PreviewView>(R.id.cameraView)

            val preview = Preview.Builder().build().also {
                it.setSurfaceProvider(cameraView?.surfaceProvider)
            }

            imageCapture = ImageCapture.Builder().build()

            val cameraSelector = CameraSelector.DEFAULT_FRONT_CAMERA

            try {
                if (allPermissionsGranted()) {
                    cameraProvider.unbindAll()
                    cameraProvider.bindToLifecycle(
                        this, cameraSelector, preview, imageCapture
                    )
                } else {
                    // 권한이 없을 경우 권한 요청
                    ActivityCompat.requestPermissions(
                        requireActivity(), REQUIRED_PERMISSIONS, REQUEST_CODE_PERMISSIONS
                    )
                }
            } catch (exc: Exception) {
                Log.e(TAG, "Use case binding failed", exc)
            }
        }, ContextCompat.getMainExecutor(requireContext()))
    }

    private fun takePhoto() {
//        deletePhotoFile()

        photoFile = File(
            outputDirectory,
            SimpleDateFormat(FILENAME_FORMAT, Locale.US)
                .format(System.currentTimeMillis()) + ".jpg"
        )

        val outputOptions = ImageCapture.OutputFileOptions.Builder(photoFile).build()

        imageCapture?.takePicture(
            outputOptions, ContextCompat.getMainExecutor(requireContext()), object : ImageCapture.OnImageSavedCallback {
                override fun onError(exc: ImageCaptureException) {
                    Log.e(TAG, "Photo capture failed: ${exc.message}", exc)
                }

                override fun onImageSaved(outputFileResults: ImageCapture.OutputFileResults) {
                    val savedUri = photoFile.absolutePath
                    val msg = "Photo capture succeeded: $savedUri"
                    Log.d(TAG, msg)

                    requireActivity().runOnUiThread {
                        val bitmap = BitmapFactory.decodeFile(savedUri)
                        val imageView = view?.findViewById<ImageView>(R.id.capturedImageView)
                        imageView?.setImageBitmap(bitmap)
                        imageView?.visibility = View.VISIBLE
                    }
                }
            })
    }

    private fun sendPhoto() {
        val client = BaseApi.getInstance().create(SendPhotoApi::class.java)
        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        val customerId = sharedPreferences.getString("customerId", null)

        // 사진 파일을 생성하여 MultipartBody.Part 객체로 변환
        val requestFile = RequestBody.create(MediaType.parse("image/jpeg"), photoFile)
        val photoPart = MultipartBody.Part.createFormData("photo", photoFile.name, requestFile)

        // 지워야됨
        view?.findNavController()?.navigate(R.id.action_photo_to_photoCheckFragment)
        //


        CoroutineScope(Dispatchers.IO).launch {
            try {
                val response = client.sendPhoto(photoPart)
                if (response.success) {
                    // 성공적으로 업로드된 경우
                    Log.d(TAG, "Photo uploaded successfully")
//                    deletePhotoFile()
                    view?.findNavController()?.navigate(R.id.action_photo_to_photoCheckFragment)
                } else {
                    // 업로드 실패한 경우
                    Log.e(TAG, "Failed to upload photo: ${response.message}")
                }
            } catch (e: Exception) {
                // 네트워크 오류 등으로 업로드에 실패한 경우
                Log.e(TAG, "Error uploading photo: ${e.message}")
            }
        }
    }

    private fun getOutputDirectory(): File {
        val mediaDir = requireActivity().externalMediaDirs.firstOrNull()?.let {
            File(it, resources.getString(R.string.app_name)).apply { mkdirs() }
        }
        return if (mediaDir != null && mediaDir.exists())
            mediaDir else requireActivity().filesDir
    }

    private fun allPermissionsGranted() = REQUIRED_PERMISSIONS.all {
        ContextCompat.checkSelfPermission(
            requireContext(), it
        ) == PackageManager.PERMISSION_GRANTED
    }

    override fun onDestroy() {
        super.onDestroy()
        cameraExecutor.shutdown()
    }

    companion object {
        private const val TAG = "CameraXBasic"
        private const val FILENAME_FORMAT = "yyyy-MM-dd-HH-mm-ss-SSS"
        private const val REQUEST_CODE_PERMISSIONS = 10
        private val REQUIRED_PERMISSIONS = arrayOf(Manifest.permission.CAMERA)
    }

    private fun deletePhotoFile() {
        if (photoFile.exists()) {
            val deleted = photoFile.delete()
            if (deleted) {
                Log.d(TAG, "Photo file deleted successfully")
            } else {
                Log.e(TAG, "Failed to delete photo file")
            }
        } else {
            Log.d(TAG, "Photo file does not exist")
        }
    }
}