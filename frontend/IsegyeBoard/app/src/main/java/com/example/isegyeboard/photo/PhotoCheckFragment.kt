package com.example.isegyeboard.photo

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import com.bumptech.glide.Glide
import com.bumptech.glide.request.target.DrawableImageViewTarget
import com.example.isegyeboard.R
import com.example.isegyeboard.databinding.FragmentPhotoCheckBinding
import com.example.isegyeboard.databinding.FragmentRecommendBinding

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

        Glide.with(this)
            .load(R.drawable.loading)
            .into(DrawableImageViewTarget(binding.photocheckView))

        binding.photoCheckBack.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }
        binding.reCaptureButton.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }
    }
}