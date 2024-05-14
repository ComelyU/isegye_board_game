package com.accio.isegye.customer.service;

import com.accio.isegye.common.entity.Base64MultipartFile;
import com.accio.isegye.common.service.S3Service;
import com.accio.isegye.customer.dto.CreateCustomerRequest;
import com.accio.isegye.customer.dto.CreateImageRequest;
import com.accio.isegye.customer.dto.CustomerResponse;
import com.accio.isegye.customer.entity.Customer;
import com.accio.isegye.customer.repository.CustomerRepository;
import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.game.entity.Game;
import com.accio.isegye.store.repository.RoomRepository;
import com.accio.isegye.store.repository.StoreRepository;
import com.amazonaws.services.s3.AmazonS3Client;
import java.io.IOException;
import java.time.Duration;
import java.time.LocalDateTime;
import lombok.RequiredArgsConstructor;
import org.apache.tomcat.util.codec.binary.Base64;
import org.modelmapper.ModelMapper;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.multipart.MultipartFile;

@Service
@RequiredArgsConstructor
public class CustomerServiceImpl implements CustomerService{

    private final CustomerRepository customerRepository;
    private final RoomRepository roomRepository;
    private final StoreRepository storeRepository;
    private final ModelMapper modelMapper;
    private final AmazonS3Client amazonS3Client;
    private final S3Service s3Service;

    @Value("${cloud.aws.s3.bucket}")
    private String bucketName;

    private String uri = "https://k10a706.p.ssafy.io/ai/swap";

    private CustomerResponse getCustomerResponse(Customer customer){
        return modelMapper.map(customer, CustomerResponse.class);
    }

    @Override
    @Transactional
    public CustomerResponse createCustomer(int roomId, CreateCustomerRequest request) {
        Customer customer = Customer.builder()
            .room(roomRepository.findById(roomId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "No Room matching: " + roomId)))
            .isTheme(request.getIsTheme())
            .peopleNum(request.getPeopleNum())
            .build();

        Customer save = customerRepository.save(customer);

        return getCustomerResponse(save);
    }

    @Override
    @Transactional
    public int endCustomer(int customerId) {
        Customer customer = customerRepository.findById(customerId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "No Customer matching: " + customerId));

        LocalDateTime startTime = customer.getStartTime();
        LocalDateTime endTime = LocalDateTime.now();
        Duration duration = Duration.between(startTime, endTime);
        long hourDifference = duration.toHours(); // 올림

        int hourFee = storeRepository.findHourFeeById(customer.getRoom().getStore().getId());

        customer.setEndTime(endTime);
        customer.setRoomFee((int)hourDifference * hourFee * customer.getPeopleNum());
        Customer update = customerRepository.save(customer);

        Integer menuFee = customerRepository.getMenuFeeByCustomerId(customerId);
        if(menuFee == null){
            menuFee = 0;
        }

        return update.getRoomFee() + menuFee;
    }

    @Override
    @Transactional
    public Integer toggleTheme(int customerId) {
        Customer customer = customerRepository.findById(customerId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "No Customer matching: " + customerId));
        int isTheme = customer.getIsTheme();
        customer.setIsTheme( (isTheme+1)%2 ); // 테마 사용 여부 전환

        Customer update = customerRepository.save(customer);

        return update.getIsTheme();
    }

    @Override
    public Integer findRoom(int customerId) {
        return roomRepository.findByCustomerId(customerId).getId();
    }

    @Override
    public String getTheme(int customerId) {
        Customer customer = customerRepository.findById(customerId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "No Customer matching: " + customerId));

        if (customer.getOrderGameList().isEmpty()
            || customer.getOrderGameList().get(customer.getOrderGameList().size()-1).getOrderStatus()==1){
            //주문한 적이 없거나 마지막 주문이 회수 상태이면
            return "Default";
        }

        Game game = customer.getOrderGameList().get(customer.getOrderGameList().size()-1).getStock()
            .getGame();

        return game.getTheme().getThemeType();
    }

    @Override
    @Async
    public String swapFace(int customerId, MultipartFile sourceFile){
        validateFileExists(sourceFile);

//        String themeType = getTheme(customerId);
        String themeType = "testType";

        String imgUrl = "";
        String themeImgUrl = "gameTheme/" + themeType + "/" + themeType + ".jpg";

        try {
            String themeFile = s3Service.downloadToBase64(themeImgUrl);

            RestTemplate restTemplate = new RestTemplate();
            CreateImageRequest imageRequest = CreateImageRequest.builder()
                .sourceFile(Base64.encodeBase64String(sourceFile.getBytes()))
                .themeFile(themeFile)
                .build();

            String resultImg = restTemplate.postForObject(uri, imageRequest, String.class);

            MultipartFile resultFile = convertBase64ToImage(resultImg, "resultImg");

            imgUrl = uploadFileToS3(resultFile, "image/", "ai");
        }catch (IOException e){
            throw new CustomException(ErrorCode.IO_ERROR, "S3 service IO Error");
        }

        return imgUrl;
    }

    private void validateFileExists(MultipartFile file){
        if(file.isEmpty()){
            throw new CustomException(ErrorCode.REQUEST_BODY_MISSING_ERROR, "File not exist");
        }
    }

    private String uploadFileToS3(MultipartFile file, String mimeTye, String dirName) {
        String fileContentType = file.getContentType();
        if(fileContentType == null || !fileContentType.startsWith(mimeTye)) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "Mime type is not supported in this request");
        }

        try {
            return s3Service.upload(file, dirName);
        } catch (IOException e) {
            throw new CustomException(ErrorCode.IO_ERROR, "I/O Exception while saving file to S3");
        }
    }

    public static MultipartFile convertBase64ToImage(String base64String, String fileName) throws IOException {
        // Decode Base64 string to byte array
        byte[] decodedBytes = Base64.decodeBase64(base64String);

        return new Base64MultipartFile(decodedBytes, fileName, fileName, "image/jpg");
    }
}
