package com.accio.isegye.common.service;

import com.amazonaws.AmazonServiceException;
import com.amazonaws.SdkClientException;
import com.amazonaws.services.s3.AmazonS3Client;
import com.amazonaws.services.s3.model.GetObjectRequest;
import com.amazonaws.services.s3.model.ObjectMetadata;
import com.amazonaws.services.s3.model.PutObjectRequest;
import com.amazonaws.services.s3.model.S3Object;
import com.amazonaws.services.s3.model.S3ObjectInputStream;
import com.amazonaws.util.Base64;
import com.amazonaws.util.IOUtils;
import java.io.IOException;
import java.net.URLEncoder;
import java.util.UUID;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.multipart.MultipartFile;

@Service
@RequiredArgsConstructor
@Transactional
@Slf4j
public class S3Service {

    @Value("${cloud.aws.s3.bucket}")
    private String bucket;

    private final AmazonS3Client amazonS3Client;

    // S3 Upload
    public String upload(MultipartFile multipartFile, String dirName) throws IOException {
        ObjectMetadata metadata = new ObjectMetadata();
        metadata.setContentType(multipartFile.getContentType());
        metadata.setContentLength(multipartFile.getSize());

        String fileName = createFileName(dirName, multipartFile.getOriginalFilename());

        PutObjectRequest request = new PutObjectRequest(bucket, fileName , multipartFile.getInputStream(), metadata);

        try {
            amazonS3Client.putObject(request);
        } catch (AmazonServiceException e) {
//            e.printStackTrace();
            log.error("AmazonServiceException occurred while uploading file", e);
        } catch (SdkClientException e) {
//            e.printStackTrace();
            log.error("SdkClientException occurred while uploading file", e);
        }

        return amazonS3Client.getUrl(bucket, fileName).toString();
    }

    // Create File Name
    private String createFileName(String dirName, String fileName) {
        return String.format("%s/%s", dirName, UUID.randomUUID() + "_" + fileName);
    }

    public String downloadToBase64(String key) throws IOException {
        GetObjectRequest getObjectRequest = new GetObjectRequest(bucket, key);

        S3Object s3Object = amazonS3Client.getObject(getObjectRequest);

        S3ObjectInputStream objectInputStream = s3Object.getObjectContent();

        byte[] bytes = IOUtils.toByteArray(objectInputStream);

        return Base64.encodeAsString(bytes);
    }
}
