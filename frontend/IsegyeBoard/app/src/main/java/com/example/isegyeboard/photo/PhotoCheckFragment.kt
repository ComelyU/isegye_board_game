package com.example.isegyeboard.photo

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import com.bumptech.glide.Glide
import com.example.isegyeboard.databinding.FragmentPhotoCheckBinding
import com.google.zxing.BarcodeFormat
import com.google.zxing.MultiFormatWriter
import com.google.zxing.WriterException
import com.google.zxing.common.BitMatrix
import android.graphics.Bitmap
import android.graphics.Color

class PhotoCheckFragment : Fragment() {

    private lateinit var binding : FragmentPhotoCheckBinding

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding = FragmentPhotoCheckBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        val imageUrl = arguments?.getString("imageUrl")
        println("이미지 받아와 짐 $imageUrl")
        Glide.with(this)
            .load(imageUrl)
            .into(binding.photoCheckView)

        val bitMatrix = generateQRCode(imageUrl!!)
        val bitmap = toBitmap(bitMatrix)

        binding.qrImageView.setImageBitmap(bitmap)

        binding.photoCheckBack.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }
        binding.reCaptureButton.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }
    }

    // QR 코드 생성 함수
    private fun generateQRCode(text: String): BitMatrix {
        val multiFormatWriter = MultiFormatWriter()
        return try {
            multiFormatWriter.encode(text, BarcodeFormat.QR_CODE, 200, 200) // 200x200 크기의 QR 코드 생성
        } catch (e: WriterException) {
            e.printStackTrace()
            throw e
        }
    }

    // BitMatrix를 Bitmap으로 변환하는 함수
    private fun toBitmap(matrix: BitMatrix): Bitmap {
        val width = matrix.width
        val height = matrix.height
        val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565)

        for (x in 0 until width) {
            for (y in 0 until height) {
                bitmap.setPixel(x, y, if (matrix[x, y]) Color.BLACK else Color.WHITE)
            }
        }
        return bitmap
    }
}